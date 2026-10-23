README for ALDL information

To find the ALDL datastream information for your car, start with the file index.txt.  In here, you can locate your engine by displacement, year, and VIN code.  Once you've located it, you'll see that it's under a DATA STREAM heading. This heading represents the datastream for that engine, and the format of the datastream can be located in a different file.  See below for an example.

You own a 1988 Chevy C/K pickup with a 5.7L throttle body injected engine.  That engine is located in the following section:

  DATA STREAM A33
            4.3L TBI   (LB4)   (VIN=Z)   86,87,88 TRUCKS
            5.0L TBI   (L03)   (VIN=H)   87,88    TRUCKS
            5.7L TBI   (L05)   (VIN=K)   87,88    TRUCKS
            4.3L TBI   (LB4)   (VIN=Z)   91       TRUCKS

These engines all share a common datastream, called A33.  The format of datastream A33 can be found in the file A033.ds
