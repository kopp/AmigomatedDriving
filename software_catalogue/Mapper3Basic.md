Mapper3Basic


# Install on x86 system

## using deb file

If you want to use
[Mapper3Basic](http://robots.mobilerobots.com/wiki/Mapper3Basic)
on a x64 system, you'll need to install a bunch of i368 packages.  It should
suffice to download the
[deb file](http://robots.mobilerobots.com/Mapper3Basic/download/current/mapper3-basic_2.2.5-1_i386.deb)
and install it

    sudo dpkg -i mapper3-basic_2.2.5-1_i386.deb

and then run

    sudo apt-get install -f

to satisfy the dependencies.


## install dependencies manually

If that does not work, extract the deb file

    ar vx mapper3-basic_2.2.5-1_i386.deb

You'll get a file `control.tar.gz` which contains `control` with the
dependencies listed.  Install them and make sure to install the i386 version;
i.e. if you're missing libxmu6 and libstdc++5 run

    sudo apt-get install libxmu6:i386 libstdc++5:i386

You can download and extract
[this tarball](http://robots.mobilerobots.com/Mapper3Basic/download/current/Mapper3Basic-2.2.5.tgz)
and run

    Mapper3Basic-2.2.5/bin/Mapper3Basic

for every `not found` entry, try to install some package that satisfies this
dependency.


## trouble shooting: i386 packages not found

If you're unable to install i386 packages, you might need to issue

    sudo dpkg --add-architecture i386
    sudo apt-get update

See
[here](http://www.unixmen.com/enable-32-bit-support-64-bit-ubuntu-13-10-greater/)
for more info.
