### BearSwerve

BearSwerve is intended to be an all in one swerve solution including: teleop driving, simulation, trajectory following and more.  It combines the work of [SDS swerve-lib](https://github.com/SwerveDriveSpecialties/swerve-lib), gerth2's [Swerve Sim](https://github.com/wpilibsuite/allwpilib/pull/3374), and [PathPlanner](https://github.com/mjansen4857/pathplanner/releases)

### Submodule

Bearswerve is intended to be used as a GitSubmodule.  This is similar to a vendordep you may be used to with other third party libraries.  The advantage of the submodule is that the code is actually present within your code.  This makes modficiations and understanding what the submodule is doing easier.

### BearSwerve Support

While BearSwerve has currently only been tested on MK4 modules as mentioned above it borrows code from other established repositories and at least a portion of this code will probably make it into WPILib in the near future.

BearSwerve is intended and generic enough to support many different swerve modules and motor / hardware configurations with those modules.

### Known Issues

Neo motors don't work in simulation.  I have had a support request open with REV for nearly a month and have gotten no response.  I am dependant on them.

