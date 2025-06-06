Network Support on Linux
========================

The simulation uses the TUN/TAP driver under Linux to provide network support.
It can operate in one of two modes: host routed, or bridged.  In the host
routed case no special configuration is necessary, but by default the
simulation will only be accessible to the host on which it runs.

Bridge mode is recommended where possible.  It requires slightly more effort
to set up, but is much more flexible, and is likely to be easier to maintain
in the end.

Host Route Mode
---------------

If CONFIG_SIM_NET_HOST_ROUTE is enabled, the simulation will create and
maintain a host route from the assigned IP address to the instance's tap
device.  This route will be updated if the application changes the
simulation's IP address.  Note that you will not see the simulation's IP
address on the TAP device if you run ifconfig on the host.

No special setup is required.  Simply assign your simulation a free IP address
on the same network as your host, and everything will Just Work.  Note that if
you assign an IP that is already in use on your network, your host won't be
able to see it until the simulation is stopped.  The host route will force all
traffic destined for that IP to be sent to the tap interface.

.. note::
   If you configure an IP address that is not on the same subnet as your
   host, additional manual setup will be required. A helper script,
   `tools/simhostroute.sh` is provided that can do this setup on Linux.
   On Windows or macOS using host route mode is not recommended.

Recent versions of Linux require setting kernel capabilities to allow the nuttx
executable access to the tap network driver. You can see more about the tun/tap
driver requiring Linux capabilities here:

https://github.com/torvalds/linux/blob/master/Documentation/networking/tuntap.txt

The `boards/sim/sim/sim/configs/tcpblaster/defconfig` is known to work in this
configuration.

To compile:

.. code-block:: bash

    $ ./tools/configure.sh sim:tcpblaster
    $ make menuconfig  # optional, to adjust configuration
    $ make clean; make

You can do the following after compiling the NuttX simulator:

On Linux:

.. code-block:: bash

    $ # necessary on recent Linux distributions
    $ sudo setcap cap_net_admin+ep ./nuttx
    $ # set up the host route and IP tables rules
    $ # replace ens33 with your Ethernet or wireless interface
    $ sudo ./tools/simhostroute.sh ens33 on
    $ # start the NuttX simulator
    $ ./nuttx

On the NuttX Simulator:

.. code-block:: bash

    nsh> # replace or omit dns if needed, IPv6 line is optional
    nsh> ifconfig eth0 inet6 fc00::2/112 dns 2001:4860:4860::8888
    nsh> ifconfig eth0 10.0.1.2 dns 8.8.8.8
    nsh> ifup eth0

On Linux:

.. code-block:: bash

    $ # is nuttx up?
    $ ping 10.0.1.2

Bridge Mode
-----------

Basic Usage
-----------
If CONFIG_SIM_NET_BRIDGE is enabled, the simulation's tap interface will
automatically be added to the Linux bridge device specified by the
CONFIG_SIM_NET_BRIDGE_DEVICE configuration option.  Note that this MUST be a
pre-existing bridge device, or the initialization will fail.  The simulation
will NOT create the bridge for you.

To create the bridge, first install the bridge utilities package for your
platform (the net-tools RPM in RedHat, for example).  Then execute a
command like the following:

.. code-block:: bash

  # ip link add nuttx0 type bridge

This will create the nuttx0 bridge.  Once created, the bridge may be used by
one or more simulations.  You only need one bridge per host; if you start
multiple simulations, they will all be added to the same bridge and can talk
amongst themselves.

Option 1: Routing Local Traffic to the Bridge
---------------------------------------------
If you want the host to be able to talk to the simulator, you will
also need to assign the bridge an IP address (this will be the default
gateway you assign to the simulator) and add a network route.  Note
that the subnet chosen should not already be in use.  For example, if
you want to use the 172.26.23.0/24 subnet for your simluations, you
would do something like the following:

.. code-block:: bash

  # ip link add nuttx0 type bridge
  # ifconfig nuttx0 172.26.23.1/24

The standard Linux ifconfig utility will automatically add the appropriate
network route, so no further effort is needed.

Option 2: Live Network Access
-----------------------------
There are two main methods of giving the simulator access to your network
at large.  One is to set up your Linux host as a router and configure your
network so that it knows where to find the appropriate subnet.  This is far
too complex for most use cases, so you can safely ignore it unless you have
specific needs.

The recommended method is to add a real interface to the bridge you're using
with NuttX.  For example, if you have a secondary eth1 interface on your host,
you can simply connect it to the network you want your simulations to access,
and run the following command:

.. code-block:: bash

  # ip link set eth1 master nuttx0

From that point on, your simulations will be directly connected to the same
network as your eth1 interface.  Note that your bridge will generally not need
an IP address in this case.

If you only have a single interface, you can configure your system so that eth0
(or other primary interface) is on the bridge.  To do this, you would execute
commands like the following from the system console:

.. code-block:: bash

  # ip link add nuttx0 type bridge
  # ip link set eth0 master nuttx0
  # ifconfig nuttx0 <host-ip-address/netmask>
  # route add -net default gw ...

The rest of your network configuration would remain the same; your host's IP
address has simply moved from being assigned directly to the ethernet interface,
to being assigned to the bridge that contains that interface.  The connection
will operate as normal.  NuttX simulations will join the bridge as with the
previous example.

In either of the live access scenarios presented here, the default gateway you
configure in your simluation should be the normal one for the network you're
accessing, whether or not the bridge has an IP address.  The bridge is acting
as an ethernet hub; your simluation has direct access to the normal gateway as
if the simluation were a device physically connected to the network.

Configuring at Startup
----------------------
Most Linux distributions have a mechanism for configuring a bridge at startup.
See your distribution's documentation for more information.

Setup Script
------------

There is a script, `tools/simbridge.sh` that will do the setup for you.

Notes
-----

  - Users of VMware ESXi should be aware that the bridge will place the contained
    ethernet interface into promiscuous mode (don't ask me why).  ESXi will
    reject this by default, and nothing will work.  To fix this, edit the
    properties of the relevant vSwitch or VLAN, select the Security tab, and
    set "Promiscuous Mode" to "Accept".

    If anyone knows a better way to deal with this, or if I'm misunderstanding
    what's happening there, please do tell.

    I don't know if VMware's consumer products have similar issues or not.

  - tools/simbridge.sh could make the bridge setup easier:

      # tools/simbridge.sh eth0 on
      # tools/simbridge.sh eth0 off

-- Steve <steve@floating.io>
   http://floating.io
