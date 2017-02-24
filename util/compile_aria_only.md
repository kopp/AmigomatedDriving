How to compile tools witih Aria only (no RosAria)


If you have a tool that uses Aria and not RosAria, such as
[wanderer](http://www.eecs.yorku.ca/course_archive/2009-10/W/4421/doc/pioneer/aria/wander_8cpp-example.html)
you can compile it easily using

    g++ wanderer.cpp -o wanderer -lAria

(when Aria is instaled sucessfully).

To run it, you will often need to specify some command line flags.  For example

    ./wanderer -rh 10.0.126.12
