# Anatomy evidence and modeling decisions

Species: **Gnathonemus petersii**, supported by source asset names and the project README. This is a mormyrid, not a gymnotiform knifefish. Retrieved 2026-09-30. The model is an external artistic asset, not a CT scan; no internal bones or organs can be reconstructed from it.

## Size and proportions

[Stiassny, Teugels & Hopkins, Fresh and Brackish Water Fishes of Lower Guinea, Mormyridae chapter, p. 279](https://horizon.documentation.ird.fr/exl-doc/pleins_textes/ed-06-08/010044400.pdf) gives a maximum of 350 mm standard length, body depth 25–28% SL, head length 20–25% SL, dorsal base 20–21% SL, anal base 27–28% SL, and peduncle length about 17% SL. It reports 27–29 dorsal rays and 34–36 anal rays. These are biological reference counts, not individually resolved features in this mesh. Per the revised request, dorsal and anal fins are each represented by one whole rigid mesh.

The chosen full mesh length is 200 mm, including the chin projection. It is **not** 200 mm SL; the caudal fin and projecting appendage change the definition. No age, sex, or maturity is inferred from this scale. Proportions are measured from the supplied mesh and retained rather than deforming the artistic asset to fit population statistics.

[Zeleke 2025, Morphological variations of Gnathonemus petersii, Fisheries and Aquatic Sciences 28:372–381](https://www.e-fas.org/archive/view_article?pid=fas-28-6-372) examines 119 museum specimens and distinguishes standard length, chin lobe, pelvic and pectoral fins, median-fin lengths and ray counts. This supports representing the small paired pelvic fins visible in this mesh. A website assertion that the species lacks pelvic fins was rejected in favor of this primary study.

## Spine, electric organ and chin

[Koch, Kirschbaum & Moritz 2023, Journal of Anatomy, DOI 10.1111/joa.13935](https://pmc.ncbi.nlm.nih.gov/articles/PMC10641036/) identifies paired dorsal and ventral Gemminger bones adjacent to the caudal-peduncle electric organ, extending anteriorly below median-fin bases. Its ontogenetic work indicates origin from fin stays. This motivates small peduncular bending, rather than an eel-like flexible tail. The organ is identified as a region only; no electrical field simulation is included.

A verified total vertebral count and quantitative regional vertebral ROM for **this species** were not found in the consulted sources. We do not substitute counts from another mormyrid. Fourteen rigid regions are numerical discretization, **not fourteen vertebrae**. Regional yaw limits (initial trunk 2°, posterior 3°, peduncle 1° per joint) and pitch 0.5° are engineering assumptions, further reduced by geometry tests. The reference-motion revision permits ±1.432° posterior yaw and ±0.716° anterior/peduncle yaw on the existing meshes, after individual-axis and combined yaw/pitch seam checks. These are geometric engineering limits, not measured vertebral ROM. No body roll or unsupported neck joint is introduced. The head, pectoral girdle and jaw remain one rigid block. The sensory chin appendage is a separate segment with yaw, pitch and roll hinges sharing a pivot, as requested. Its current yaw/pitch/roll limits (±45°/±45°/±30°) are engineering choices for active scanning, not verified anatomical ROM. The current rigid ball/socket chin enforces socket-rim coverage without reducing these ranges; surface tangency and freedom from intersection at extreme bends are not guaranteed.

## Swimming and fins

[Lannoo & Lannoo 1993, Why do electric fishes swim backwards?](https://www.ikhebeenvraag.be/mediastorage/FSDocument/56/Lannoo-157.pdf) discusses carangiform movements in Gnathonemus, semi-stiff body axes and a rigid electric-organ peduncle, as well as backward probing movements. This model combines modest posterior body motion, caudal and pectoral strokes, while holding the single rigid dorsal/anal meshes neutral. It does not describe this species as a long-ribbon-fin knifefish.

[Greisman & Moller 2005, The anal fin complex in a weakly discharging electric fish, DOI 10.1111/j.0022-1112.2005.00608.x](https://onlinelibrary.wiley.com/doi/10.1111/j.0022-1112.2005.00608.x) reports sexual differences in anal-fin rays and proximal pterygiophores. This asset's sex is unknown. The complete dorsal and anal fins each have one low-range longitudinal hinge and stay neutral by default. Fin rays, ray spacing, membrane flexibility and muscles are not individually resolved. Pectoral ranges and all mechanical stiffnesses, damping and gains are uncalibrated assumptions.

Forward/backward motion, hover commands, turning and body-wave comparisons are controller experiments. The simplified median fins are not used to synthesize ribbon waves. Body and caudal phase, pectoral strokes and an active three-axis chin scan are configurable. This mesh has no long ribbon fin. Hover is open-loop, not proven station keeping.

## Geometry and hydrodynamics constraints

Concentric spherical cut patches have rotationally invariant supporting surfaces, but their *finite boundaries* and the nonspherical exterior skin are not invariant. Matching spheres cannot guarantee no exterior step at arbitrary bend. Tests sample actual neighboring seam boundaries, nearest triangle surfaces and penetration, then conservatively reduce body ROM. Chin ROM is separately configurable; the current ball/socket attachment enforces socket-rim coverage throughout sampled scanning poses. Tessellated sphere facets introduce additional small errors. Smooth membrane deformation is not modeled by the whole rigid fins.

[MuJoCo official fluid-force documentation](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) describes stateless ellipsoid-based drag, lift and added-mass approximations, not CFD, wakes or fin-to-fin hydrodynamic coupling. Use `implicitfast` integration. Fluid density does not supply hydrostatic buoyancy: gravity-on operation applies displaced-volume buoyancy at each segment COM, assuming fully submerged uniform-density tissue. Collision proxies are not used as displaced volumes. Overlap mass is corrected using the repaired source volume. A neutral passive test verifies the implementation.

## Active chin motion revision

[Amey-Özel et al. (2015), More a finger than a nose](https://pubmed.ncbi.nlm.nih.gov/25388854/) describes motor and sensory innervation of the highly mobile Schnauzenorgan. [von der Emde et al. (2008), Active electrolocation](https://pubmed.ncbi.nlm.nih.gov/18992334/) describes rhythmic lateral searching and directed probing movements. These sources support substantial active movement; the consulted abstracts do not establish numerical yaw, pitch, or roll limits or a 1 Hz scanning frequency. The simulation uses those adjustable engineering settings without claiming empirical calibration.
