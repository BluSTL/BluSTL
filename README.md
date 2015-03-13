# BluSTL
BluSTL (pronounced "blue steel") is a MATLAB toolkit for automatically generating hybrid controllers from specifications written in Signal Temporal Logic.

# Installing

BluSTL depends on YALMIP, which is best obtained with the Multi-Parametric Toolbox, or MPT3, see http://control.ee.ethz.ch/~mpt/3/Main/Installation.

Most experiments have been done with the Gurobi solver as
back-end, though other solvers might work as well. For the example to work without modifications
though, Gurobi needs to be installed and configured for Matlab. See http://www.gurobi.com.

Once YALMIP (or MPT3), the only thing to do  is to add the path BluSTL/src to Matlab paths.

# Testing

Go into BluSTL/examples/tutorials and run tutorial1.

# Contact us

Questions are to be addressed to donze@berkeley.edu or vasu@caltech.edu.
