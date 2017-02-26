External ros packages that are not provided as binary installs.


# Add new external package

Please use `rosdep resolve <package>` to check, whether the package already
exists as pre-compiled binary.  If so, just mention the package in your
`package.xml`, so that it is installed by rosdep.  If not, add the repository
of this package to this folder by using `git submodule add`; for example

    git submodule add https://github.com/Sahloul/ar_sys.git ar_sys

If the package is not provided as git repository (check, whether there is a git
mirror), add an empty folder for that package `<package>` and a script
`get_<package>.sh` in this folder and add the `<package>` folder to the
`.gitignore` file.  The `get_<package>.sh` should download and extract the
package's contents into the folder `<package>`.
