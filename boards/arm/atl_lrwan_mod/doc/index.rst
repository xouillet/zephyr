.. _atl_lrwan_mod_board:

ATL-Electronics LoRaWAN Module
##############################

Overview
********

Supported Features
==================

The board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| PINMUX    | on-chip    | pinmux                              |
+-----------+------------+-------------------------------------+
| I2C       | on-chip    | i2c                                 |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | spi                                 |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| RTC       | on-chip    | counter                             |
+-----------+------------+-------------------------------------+
| TRNG      | on-chip    | true random number generator        |
+-----------+------------+-------------------------------------+
| EEPROM    | on-chip    | eeprom                              |
+-----------+------------+-------------------------------------+
| FLASH     | on-chip    | flash                               |
+-----------+------------+-------------------------------------+
| LoRa      | on-module  | sx1276                              |
+-----------+------------+-------------------------------------+

Other hardware features are not yet supported on this Zephyr port.

The default configuration can be found in the defconfig file:
``boards/arm/atl_lrwan_mod/atl_lrwan_mod_defconfig``


Connections and IOs
===================

Flashing an application
-----------------------

Here is an example for the :ref:`hello_world` application.

Connect the ATL_LRWAN_MOD board to a STLinkV2 to your host computer using the
USB port, then run a serial host program to connect with your board.
For example:

.. code-block:: console

   $ minicom -D /dev/ttyACM0

Then build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: atl_lrwan_mod
   :goals: build flash

You should see the following message on the console:

.. code-block:: console

   $ Hello World! arm
