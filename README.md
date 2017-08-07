# Work Envelope Approximator for Robot Arms
This is a MATLAB program created to numerically approximate the work envelopes of generic robot manipulators. It works by iterating through the forward kinematics solution with all possible combinations of joint angles, and counting the number of unique and total points that can be reached. The results are also plotted in 3-dimensions.

The program makes use of the Parallel Computing Toolbox to find the solutions more quickly.

This program was created by Baran Usluel during high school, to provide values for a section of a research project on optimizing the number of degrees of freedom (DOF) in robots. The research project was done to fulfill the Extended Essay component of the International Baccalaureate Diploma Programme. The relevant paper can be found here:

http://www.prism.gatech.edu/~busluel3/files/ib-extended-essay.pdf