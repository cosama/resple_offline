# Known issues

## Heterogeneous-core divergence

H420 replay depends on Intel core class. Two P-core runs reproduce one exact
trajectory, while two E-core runs reproduce another. The trajectories separate
by up to 1.69 m despite identical inputs and processing counts.

The initial difference is roundoff-sized. A short odometric lag around
525--550 s then establishes a translated trajectory frame, mostly along the
hallway direction. Tree-rebuild quiescence and serial Eigen execution do not
remove the split.

RESPLE's data-dependent IEKF convergence and reassociation schedule amplifies
the difference. Five fixed reassociation passes bound P/E separation to
`2.9e-11` m,
but are not an acceptable fix: other datasets move by up to 89.6 m and runtime
increases by as much as 3.34x.

Until the first convergence-decision mismatch is isolated, use one CPU core
class for repeatability. Do not use fixed full convergence as a workaround.
