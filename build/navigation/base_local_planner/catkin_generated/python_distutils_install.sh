#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/dvij5420/catkin_ws/src/navigation/base_local_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/dvij5420/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/dvij5420/catkin_ws/install/lib/python2.7/dist-packages:/home/dvij5420/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/dvij5420/catkin_ws/build" \
    "/home/dvij5420/anaconda2/bin/python2" \
    "/home/dvij5420/catkin_ws/src/navigation/base_local_planner/setup.py" \
     \
    build --build-base "/home/dvij5420/catkin_ws/build/navigation/base_local_planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/dvij5420/catkin_ws/install" --install-scripts="/home/dvij5420/catkin_ws/install/bin"
