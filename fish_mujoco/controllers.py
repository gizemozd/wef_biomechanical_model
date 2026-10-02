"""Bounded open-loop CPG experiments; no artificial propulsion forces."""
import json
import numpy as np
import mujoco
from common import ROOT

class Controller:
    def __init__(self,model,c,behavior='forward',overrides=None):
        self.m=model;self.c=c;self.behavior=behavior
        self.p={**c['controller'],**c['controller'].get('gaits',{}).get(behavior,{}),**(overrides or {})}
        self.spec=json.loads((ROOT/'assets/joints.json').read_text())
        meta=json.loads((ROOT/'assets/segments.json').read_text());self.segs={s['name']:s for s in meta['segments']}
        self.L=c['source']['length_m']
        self.aids=np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'a_'+j['name']) for j in self.spec])
        self.ranges=model.actuator_ctrlrange[self.aids]
        self.s=np.array([(.5*self.L-self.segs[j['body']]['origin'][0])/self.L for j in self.spec])
        self.kind=np.array([j['kind'] for j in self.spec])
        self.names=[j['name'] for j in self.spec]
        body_s=self.s[self.kind=='body']
        self.body_span=(body_s.min(),np.ptp(body_s))
    def __call__(self,t):
        p=self.p;f=p['frequency_hz'];wave=p['wavelength_bl'];amp=np.deg2rad(p['amplitude_deg'])
        ramp=min(1,t/max(p['ramp_s'],1e-6));ramp=.5-.5*np.cos(np.pi*ramp)
        result=np.zeros(len(self.spec));direction=-1 if self.behavior=='backward' else 1
        dynamic_yaw=0.0
        phase=2*np.pi*(f*t-direction*self.s/wave)
        for k,j in enumerate(self.spec):
            kind=j['kind'];lim=self.ranges[k,1];s=self.s[k]
            if kind in ('anal','dorsal'):
                # Each median fin is now a single mesh; neutral by default.
                result[k]=np.deg2rad(p['median_fin_amplitude_deg'])*np.sin(2*np.pi*f*t)
            elif kind=='chin':
                offset={'yaw':0,'pitch':np.pi/2,'roll':np.pi}[j['name'].split('_')[-1]]
                result[k]=lim*p['chin_amplitude_fraction']*np.sin(2*np.pi*p['chin_frequency_hz']*t+offset)
            elif kind=='body':
                if j['name'].endswith('yaw'):
                    u=np.clip((s-self.body_span[0])/self.body_span[1],0,1)
                    floor=p.get('body_amplitude_floor',0.2)
                    fraction=p['body_amplitude_fraction']*(floor+(1-floor)*u*u)
                    if p['profile']=='knifefish_rigid_trunk' and s<.8:fraction*=.05
                    result[k]=lim*fraction*np.sin(2*np.pi*(f*t-direction*s/wave))
                    # Compensate body/fin tracking lag when a faster gait is used.
                    dynamic_yaw+=lim*fraction*np.sin(phase[k]+np.deg2rad(p.get('caudal_phase_deg',0)))
                    if self.behavior=='turning':result[k]+=lim*p['turn_bias_fraction']
                elif self.behavior=='pitch':result[k]=.5*lim
            elif kind=='caudal':
                # Filled after body targets: the fin feathers against rear-body
                # curvature instead of pitching independently about a fixed base.
                pass
            elif kind=='pectoral':
                sign=1 if 'pecL' in j['name'] else -1
                a=np.deg2rad(p['pec_amplitude_deg'])*p['amplitude_deg']/22
                if self.behavior=='braking':a*=2
                if self.behavior=='turning' and sign<0:a*=p.get('turn_pectoral_inner_fraction',.6)
                phase_pec=2*np.pi*f*t
                if 'abduct' in j['name']:result[k]=sign*a*np.sin(phase_pec)
                elif 'protract' in j['name']:result[k]=sign*.5*a*np.cos(phase_pec)*p.get('pectoral_phase_sign',1)*(-1 if self.behavior=='backward' else 1)
                # Under left/right reflection an axial y rotation keeps its sign.
                # Feathering about the fin span controls thrust direction.
                else:result[k]=-direction*p.get('pec_pitch_fraction',.5)*a*np.sin(phase_pec)
            elif kind=='pelvic':result[k]=.12*amp*np.sin(phase[k])
        for k in np.flatnonzero(self.kind=='caudal'):
            a=np.deg2rad(p['caudal_amplitude_deg'])
            if p.get('caudal_mode','counterbend')=='counterbend':
                result[k]=np.clip(-p['caudal_counterbend_gain']*dynamic_yaw,-a,a)
            else:
                result[k]=a*np.sin(phase[k]+np.deg2rad(p.get('caudal_phase_deg',0)))
            if self.behavior=='turning':result[k]+=np.deg2rad(p.get('turn_caudal_bias_deg',1.5))
        for kind in p.get('disabled_groups',[]):result[self.kind==kind]=0
        if self.behavior=='body_undulation':result[np.isin(self.kind,['dorsal','anal'])]*=.15
        if self.behavior=='chin_scan':result[self.kind!='chin']=0
        if self.behavior=='passive':result[:]=0
        return np.clip(result*ramp,self.ranges[:,0],self.ranges[:,1])
    def apply(self,data):data.ctrl[self.aids]=self(data.time)

def simulate(model,c,behavior='forward',duration=None,overrides=None,record_fps=None):
    data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,0);ctl=Controller(model,c,behavior,overrides)
    seconds=duration or c['simulation']['duration_s'];dt=model.opt.timestep
    times=[];poses=[];vels=[];q=[];qvel=[];controls=[]
    rootid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'body_00_head')
    stride=max(1,round(1/(record_fps*dt))) if record_fps else 10
    for k in range(round(seconds/dt)+1):
        if k%stride==0:
            mujoco.mj_forward(model,data);mujoco.mj_subtreeVel(model,data);times.append(data.time);poses.append(data.subtree_com[rootid].copy());vels.append(data.subtree_linvel[rootid].copy());q.append(data.qpos.copy());qvel.append(data.qvel.copy());controls.append(data.ctrl.copy())
        if k==round(seconds/dt):break
        ctl.apply(data);mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all() or data.warning.number.sum():raise RuntimeError(f'Unstable {behavior} at t={data.time}: {data.warning.number}')
    return {'time':np.array(times),'position':np.array(poses),'velocity':np.array(vels),'qpos':np.array(q),'qvel':np.array(qvel),'ctrl':np.array(controls)}
