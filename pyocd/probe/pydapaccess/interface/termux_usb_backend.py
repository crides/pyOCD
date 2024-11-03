# Copyright (c) 2024 crides
# SPDX-License-Identifier: MIT

import logging
import threading
import platform
import errno
import queue
import os
from typing import Optional

from .interface import Interface
from .common import (
    USB_CLASS_HID,
    filter_device_by_class,
    is_known_device_string,
    is_known_cmsis_dap_vid_pid,
    generate_device_unique_id,
    )
from ..dap_access_api import DAPAccessIntf
from ....utility.timeout import Timeout

LOG = logging.getLogger(__name__)
TRACE = LOG.getChild("trace")
TRACE.setLevel(logging.CRITICAL)

try:
    import usb.backend.libusb1 as libusb1
    import usb.core
    import usb.util
except ImportError:
    IS_AVAILABLE = False
else:
    IS_AVAILABLE = True

class TermuxUSB(Interface):
    """@brief CMSIS-DAP USB interface class using pyusb for the backend."""

    isAvailable = IS_AVAILABLE

    did_show_no_libusb_warning = False

    def __init__(self, dev):
        super().__init__()
        self.vid = dev.idVendor
        self.pid = dev.idProduct
        self.product_name = dev.product or f"{dev.idProduct:#06x}"
        self.vendor_name = dev.manufacturer or f"{dev.idVendor:#06x}"
        self.serial_number = dev.serial_number \
                or generate_device_unique_id(dev.idProduct, dev.idVendor, dev.bus, dev.address)
        self.ep_out = None
        self.ep_in = None
        self.dev = dev
        LOG.debug("usb dev: %s", dev != None)
        self.intf_number = None
        self.kernel_driver_was_attached = False
        self.closed = True
        self.thread = None
        self.rx_stop_event = threading.Event()
        self.rcv_data: queue.SimpleQueue[bytes] = queue.SimpleQueue()
        self.read_sem = threading.Semaphore(0)
        self._read_thread_did_exit: bool = False
        self._read_thread_exception: Optional[Exception] = None

    def open(self):
        assert self.closed is True

        # get active config
        LOG.debug("open dev: %s", self.dev != None)
        config = self.dev.get_active_configuration()
        LOG.debug("open config")

        # Get count of HID interfaces and create the matcher object
        hid_interface_count = len(list(usb.util.find_descriptor(config, find_all=True, bInterfaceClass=USB_CLASS_HID)))
        LOG.debug("open hid count: %d", hid_interface_count)
        matcher = MatchCmsisDapv1Interface(hid_interface_count)

        # Get CMSIS-DAPv1 interface
        interface = usb.util.find_descriptor(config, custom_match=matcher)
        if interface is None:
            raise DAPAccessIntf.DeviceError(f"Probe {self.serial_number} has no CMSIS-DAPv1 interface")
        interface_number = interface.bInterfaceNumber

        # Find endpoints
        ep_in, ep_out = None, None
        for endpoint in interface:
            if endpoint.bEndpointAddress & usb.util.ENDPOINT_IN:
                ep_in = endpoint
            else:
                ep_out = endpoint

        # Detach kernel driver
        self.kernel_driver_was_attached = False
        try:
            if self.dev.is_kernel_driver_active(interface_number):
                LOG.debug("Detaching Kernel Driver of Interface %d from USB device (VID=%04x PID=%04x).",
                          interface_number, self.dev.idVendor, self.dev.idProduct)
                self.dev.detach_kernel_driver(interface_number)
                self.kernel_driver_was_attached = True
        except usb.core.USBError as e:
            LOG.warning("USB Kernel Driver Detach Failed ([%s] %s). Attached driver may interfere with pyOCD operations.", e.errno, e.strerror)
        except NotImplementedError as e:
            # Some implementations don't don't have kernel attach/detach
            LOG.debug("Probe %s: USB kernel driver detaching is not supported. Attached HID driver may interfere with pyOCD operations.", self.serial_number)

        # Explicitly claim the interface
        try:
            usb.util.claim_interface(self.dev, interface_number)
        except usb.core.USBError as exc:
            raise DAPAccessIntf.DeviceError(f"Unable to claim interface for probe {self.serial_number}") from exc

        # Update all class variables if we made it here
        self.ep_out = ep_out
        self.ep_in = ep_in
        self.intf_number = interface_number

        # Start RX thread as the last step
        self.closed = False
        self.start_rx()

    def start_rx(self):
        # Flush the RX buffers by reading until timeout exception
        try:
            while True:
                self.ep_in.read(self.ep_in.wMaxPacketSize, 1)
        except usb.core.USBError:
            # USB timeout expected
            pass

        # Start RX thread
        thread_name = f"CMSIS-DAPv1 receive ({self.serial_number})"
        self.thread = threading.Thread(target=self.rx_task, name=thread_name)
        self.thread.daemon = True
        self.thread.start()

    def rx_task(self):
        try:
            while not self.rx_stop_event.is_set():
                self.read_sem.acquire()
                if not self.rx_stop_event.is_set():
                    read_data = self.ep_in.read(self.ep_in.wMaxPacketSize,
                            timeout=self.DEFAULT_USB_TIMEOUT_MS).tobytes()

                    # This trace log is commented out to reduce clutter, but left in to leave available
                    # when debugging rx_task issues.
                    # if TRACE.isEnabledFor(logging.DEBUG):
                    #     # Strip off trailing zero bytes to reduce clutter.
                    #     TRACE.debug("  USB IN < (%d) %s", len(read_data),
                    #                 ' '.join([f'{i:02x}' for i in read_data.rstrip(b'\x00')]))

                    self.rcv_data.put(read_data)
        except Exception as err:
            TRACE.debug("rx_task exception: %s", err)
            self._read_thread_exception = err
        finally:
            self._swo_thread_did_exit = True

    @staticmethod
    def get_all_connected_interfaces():
        """@brief Returns all the connected CMSIS-DAP devices.

        returns an array of PyUSB (Interface) objects
        """

        fd = int(os.getenv("TERMUX_USB_FD"))
        # setup library
        backend = libusb1.get_backend()
        lib, ctx = backend.lib, backend.ctx

        # extend c wrapper with android functionality
        lib.libusb_set_option.argtypes = [
            libusb1.c_void_p,
            libusb1.c_int,
        ]
        lib.libusb_wrap_sys_device.argtypes = [
            libusb1.c_void_p,
            libusb1.c_int,
            libusb1.POINTER(libusb1._libusb_device_handle),
        ]

        lib.libusb_get_device.argtypes = [libusb1.c_void_p]
        lib.libusb_get_device.restype = libusb1._libusb_device_handle

        LOG.debug("usb fd: %s, %s, %s", fd, lib, ctx)
        lib.libusb_set_option(ctx, 2);
        lib.libusb_init(libusb1.byref(ctx))
        LOG.debug("usb ctx: %s", ctx)

        # get handle from file descriptor
        handle = libusb1._libusb_device_handle()
        LOG.debug("usb fd: %s, %s", handle, libusb1.byref(handle))
        res = lib.libusb_wrap_sys_device(ctx, fd, libusb1.byref(handle))
        LOG.debug("usb fd: %s", res)
        libusb1._check(res)
        LOG.debug("usb handle: %s", handle)

        # get device (id?) from handle
        devid = lib.libusb_get_device(handle)
        LOG.debug("usb devid: %s", devid)

        # device: devid + handle wrapper
        class DummyDevice:
            def __init__(self, devid, handle):
                self.devid = devid
                self.handle = handle

        dev = DummyDevice(devid, handle)

        # create pyusb device
        device = usb.core.Device(dev, backend)
        LOG.debug("usb dev: %s", device != None)
        device._ctx.handle = dev

        # device.set_configuration()

        dev = TermuxUSB(device)
        LOG.debug("ret dev: %s", dev)
        return [dev]

    def write(self, data):
        """@brief Write data on the OUT endpoint associated to the HID interface"""

        report_size = self.packet_size
        if self.ep_out:
            report_size = self.ep_out.wMaxPacketSize

        # Trace output data before padding.
        if TRACE.isEnabledFor(logging.DEBUG):
            TRACE.debug("  USB OUT> (%d) %s", len(data), ' '.join([f'{i:02x}' for i in data]))

        data.extend([0] * (report_size - len(data)))

        self.read_sem.release()

        if not self.ep_out:
            bmRequestType = 0x21       # Host to device request of type Class of Recipient Interface
            bmRequest = 0x09           # Set_REPORT (HID class-specific request for transferring data over EP0)
            wValue = 0x200             # Issuing an OUT report
            wIndex = self.intf_number  # interface number for HID
            self.dev.ctrl_transfer(bmRequestType, bmRequest, wValue, wIndex, data,
                    timeout=self.DEFAULT_USB_TIMEOUT_MS)
        else:
            self.ep_out.write(data, timeout=self.DEFAULT_USB_TIMEOUT_MS)

    def read(self):
        """@brief Read data on the IN endpoint associated to the HID interface"""
        if self.closed:
            return b''
        elif self._read_thread_did_exit:
            raise DAPAccessIntf.DeviceError(f"Probe {self.serial_number} read thread exited unexpectedly") \
                from self._read_thread_exception

        try:
            data = self.rcv_data.get(True, self.DEFAULT_USB_TIMEOUT_S)
        except queue.Empty:
            raise DAPAccessIntf.DeviceError(f"Timeout reading from probe {self.serial_number}") from None

        # Trace when the higher layer actually gets a packet previously read.
        if TRACE.isEnabledFor(logging.DEBUG):
            # Strip off trailing zero bytes to reduce clutter.
            TRACE.debug("  USB RD < (%d) %s", len(data),
                    ' '.join([f'{i:02x}' for i in bytes(data).rstrip(b'\x00')]))

        return data

    def close(self):
        """@brief Close the interface"""
        assert self.closed is False

        LOG.debug("closing interface")
        self.closed = True
        self.rx_stop_event.set()
        self.read_sem.release()
        self.thread.join()
        self.rx_stop_event.clear() # Reset the stop event.
        self.rcv_data = queue.SimpleQueue() # Recreate queue to ensure it's empty.
        # usb.util.release_interface(self.dev, self.intf_number)
        # if self.kernel_driver_was_attached:
        #     try:
        #         self.dev.attach_kernel_driver(self.intf_number)
        #     except Exception as exception:
        #         LOG.warning('Exception attaching kernel driver: %s',
        #                         str(exception))
        # usb.util.dispose_resources(self.dev)
        # self.ep_out = None
        # self.ep_in = None
        # self.intf_number = None
        # self.kernel_driver_was_attached = False
        # self.thread = None
        # self._read_thread_did_exit = False
        # self._read_thread_exception = None

class MatchCmsisDapv1Interface:
    """@brief Match class for finding CMSIS-DAPv1 interface.

    This match class performs several tests on the provided USB interface descriptor, to
    determine whether it is a CMSIS-DAPv1 interface. These requirements must be met by the
    interface:

    1. If there is more than one HID interface on the device, the interface must have an interface
        name string containing "CMSIS-DAP".
    2. bInterfaceClass must be 0x03 (HID).
    3. bInterfaceSubClass must be 0.
    4. Must have interrupt in endpoint, with an optional interrupt out endpoint, in that order.
    """

    def __init__(self, hid_interface_count):
        """@brief Constructor."""
        self._hid_count = hid_interface_count

    def __call__(self, interface):
        """@brief Return True if this is a CMSIS-DAPv1 interface."""
        try:
            if self._hid_count > 1:
                interface_name = usb.util.get_string(interface.device, interface.iInterface)

                # This tells us whether the interface is CMSIS-DAP, but not whether it's v1 or v2.
                if (interface_name is None) or ("CMSIS-DAP" not in interface_name):
                    return False

            # Now check the interface class to distinguish v1 from v2.
            if (interface.bInterfaceClass != USB_CLASS_HID) \
                or (interface.bInterfaceSubClass != 0):
                return False

            # Must have either 1 or 2 endpoints.
            if interface.bNumEndpoints not in (1, 2):
                return False

            endpoint_attrs = [
                (usb.util.endpoint_direction(ep.bEndpointAddress),
                 usb.util.endpoint_type(ep.bmAttributes))
                 for ep in interface
            ]

            # Possible combinations of endpoints
            ENDPOINT_ATTRS_ALLOWED = [
                # One interrupt endpoint IN
                [(usb.util.ENDPOINT_IN, usb.util.ENDPOINT_TYPE_INTR)],
                # Two interrupt endpoints, first one IN, second one OUT
                [(usb.util.ENDPOINT_IN, usb.util.ENDPOINT_TYPE_INTR),
                 (usb.util.ENDPOINT_OUT, usb.util.ENDPOINT_TYPE_INTR)],
                # Two interrupt endpoints, first one OUT, second one IN
                [(usb.util.ENDPOINT_OUT, usb.util.ENDPOINT_TYPE_INTR),
                 (usb.util.ENDPOINT_IN, usb.util.ENDPOINT_TYPE_INTR)],
            ]
            if endpoint_attrs not in ENDPOINT_ATTRS_ALLOWED:
                return False

            # All checks passed, this is a CMSIS-DAPv2 interface!
            return True

        except (UnicodeDecodeError, IndexError):
            # UnicodeDecodeError exception can be raised if the device has a corrupted interface name.
            # Certain versions of STLinkV2 are known to have this problem. If we can't read the
            # interface name, there's no way to tell if it's a CMSIS-DAPv2 interface.
            #
            # IndexError can be raised if an endpoint is missing.
            return False
