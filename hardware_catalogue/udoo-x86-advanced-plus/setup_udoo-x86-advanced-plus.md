# Setup UDOO x86 Advanced Plus


# Login

Name, username and password are all `amigo`.


# Install Ubuntu

- Download the Desktop ISO, and put it onto a thumb drive.
- Connect the Udoo per cable to the internet.
- Boot from the thumb drive and select to install directly.
  - Check _Download updates while installing Ubuntu_ and _Install third-party software..._
  - The installer will probably ask whether you want to _Force UEFI installation?_.
    This happens, because Udoo starts in/with UEFI mode, but the thumb drive uses the legacy _BIOS compatibility mode_.
    Choose _Continue in UEFI mode_ to tell the installer that we are not interested to boot the installer again.
  - For _Installation type_, choose _Erase disk and install Ubuntu_, leave all the other boxes unchecked, then click _Install Now_.
    Note: If you select _Something else_, the installation will probably fail due to an Ubuntu bug.
  - The installer will install to the eMMC, `mmcblk0` and  create partitions for ESP (EFI system partiton), `/` (ext4) and a swap space.
    The latter is not really required, but due to the installation bug, you'll have to live with it...
  - Select timezone _Berlin_, kayboard layout _English (US)_ and
    - Your Name: _amigo_
    - Your computer's name: _amigo-UDOO-x86-`<num>`_ (where `<num>` is an incrementing counter (without leading zeros), starting at `1` for the first Udoo)
    - Pick a username: _amigo_
    - Choose a password: _amigo_
    - Select _Log in automatically_ and leave the other boxes/radio buttons unchecked.
  - Wait for the installation procedure to finish -- this will take (given a fairly good internet connection) about 20 min.
  - Hit _Restart now_, emove the installation medium and hit Enter to reboot the Udoo.


# Install AmigomatedDriving and dependencies

Download the `udoo_x86_setup.sh` script and execute it as normal user (who is
in the sudoers group)

    udoo_x86_setup.sh

If you want to have the content of AmigomatedDriving built and the ros-famous
`source devel/setup.bash` entry in `~/.bashrc`, then run the script as

    udoo_x86_setup.sh --build

It should ask once for _amigo_'s password (which is _amigo_) and then work for
some time.

Run `udoo_x86_setup.sh` with `--help` to get more info on what it does.


## Verbose Execution

If you want to know, what the script is doing, run it with

    bash -x udoo_x86_setup.sh
    bash -x udoo_x86_setup.sh --build # respectively

It will print the command to execute in a line starting with `+`.

To log all that happened during execution use e.g.

    bash -x udoo_x86_setup.sh --build 2>&1 | tee udoo_x86_setup.log

or to log errors to a separate file use
([explaination](https://stackoverflow.com/a/692407/2165903))

    bash -x udoo_x86_setup.sh --build > >(tee udoo_x86_setup.log) 2> >(tee udoo_x86_setup.errors >&2)

This is convenient, if you want to run the setup with minimal human
intervention -- e.g. when you combine it with `; poweroff` to turn off the Udoo
after installation/build.
