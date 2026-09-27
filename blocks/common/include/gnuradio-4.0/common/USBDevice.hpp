#ifndef GNURADIO_USB_DEVICE_HPP
#define GNURADIO_USB_DEVICE_HPP

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace gr::blocks::common {

struct USBDeviceId {
    std::uint16_t    vendorId;
    std::uint16_t    productId;
    std::string_view description;
};

struct USBDeviceInfo {
    std::uint16_t vendorId  = 0;
    std::uint16_t productId = 0;
    std::uint8_t  busNum    = 0;
    std::uint8_t  devNum    = 0;
    std::string   devPath; // "/dev/bus/usb/001/004"
    std::string   product;
    std::string   manufacturer;
    bool          accessible = false;
};

namespace detail {

inline std::uint16_t parseHex16(std::string_view s) {
    std::uint16_t result = 0;
    std::from_chars(s.data(), s.data() + s.size(), result, 16);
    return result;
}

} // namespace detail

#if defined(__linux__)

namespace detail {

inline std::string readSysfsAttr(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::string   content;
    if (file && std::getline(file, content)) {
        while (!content.empty() && (content.back() == '\n' || content.back() == '\r' || content.back() == ' ')) {
            content.pop_back();
        }
    }
    return content;
}

inline std::uint8_t parseDec8(std::string_view s) {
    std::uint8_t result = 0;
    std::from_chars(s.data(), s.data() + s.size(), result);
    return result;
}

} // namespace detail

[[nodiscard]] inline bool canAccessUSBDevice(const USBDeviceInfo& device) { return ::access(device.devPath.c_str(), R_OK | W_OK) == 0; }

inline std::vector<USBDeviceInfo> enumerateUSBDevices(std::span<const USBDeviceId> vidPidFilter = {}) {
    namespace fs = std::filesystem;

    std::vector<USBDeviceInfo> result;
    std::error_code            ec;

    for (const auto& entry : fs::directory_iterator("/sys/bus/usb/devices", ec)) {
        auto idVendorPath = entry.path() / "idVendor";
        if (!fs::exists(idVendorPath, ec)) {
            continue;
        }

        auto vid = detail::parseHex16(detail::readSysfsAttr(idVendorPath));
        auto pid = detail::parseHex16(detail::readSysfsAttr(entry.path() / "idProduct"));

        if (!vidPidFilter.empty()) {
            bool match = false;
            for (const auto& f : vidPidFilter) {
                if (f.vendorId == vid && f.productId == pid) {
                    match = true;
                    break;
                }
            }
            if (!match) {
                continue;
            }
        }

        auto busNum = detail::parseDec8(detail::readSysfsAttr(entry.path() / "busnum"));
        auto devNum = detail::parseDec8(detail::readSysfsAttr(entry.path() / "devnum"));

        USBDeviceInfo info;
        info.devPath      = std::format("/dev/bus/usb/{:03d}/{:03d}", busNum, devNum);
        info.product      = detail::readSysfsAttr(entry.path() / "product");
        info.manufacturer = detail::readSysfsAttr(entry.path() / "manufacturer");
        info.vendorId     = vid;
        info.productId    = pid;
        info.busNum       = busNum;
        info.devNum       = devNum;
        info.accessible   = canAccessUSBDevice(info);
        result.push_back(std::move(info));
    }
    return result;
}

struct USBDevice {
    using Result = std::expected<void, std::string>;

    // one bulk IN transfer of the read queue; the kernel owns it from submission until it is reaped
    struct QueuedTransfer {
        std::vector<std::uint8_t>     buffer;
        std::unique_ptr<usbdevfs_urb> urb = std::make_unique<usbdevfs_urb>();
    };
    static constexpr std::size_t kNoTransfer = std::numeric_limits<std::size_t>::max();

    int                         _fd        = -1;
    int                         _interface = -1;
    std::vector<QueuedTransfer> _queue;
    std::size_t                 _heldIndex  = kNoTransfer; // a reaped transfer not yet handed out whole
    std::size_t                 _heldOffset = 0UZ;

    USBDevice()                            = default;
    USBDevice(const USBDevice&)            = delete;
    USBDevice& operator=(const USBDevice&) = delete;
    USBDevice(USBDevice&& o) noexcept : _fd(std::exchange(o._fd, -1)), _interface(std::exchange(o._interface, -1)), _queue(std::move(o._queue)), _heldIndex(std::exchange(o._heldIndex, kNoTransfer)), _heldOffset(std::exchange(o._heldOffset, 0UZ)) {}
    USBDevice& operator=(USBDevice&& o) noexcept {
        close();
        _fd         = std::exchange(o._fd, -1);
        _interface  = std::exchange(o._interface, -1);
        _queue      = std::move(o._queue);
        _heldIndex  = std::exchange(o._heldIndex, kNoTransfer);
        _heldOffset = std::exchange(o._heldOffset, 0UZ);
        return *this;
    }
    ~USBDevice() { close(); }

    [[nodiscard]] bool isOpen() const { return _fd >= 0; }

    // ── lifecycle ───────────────────────────────────────────────────────────

    [[nodiscard]] Result open(const USBDeviceInfo& device, int interfaceNum = 0) {
        _fd = ::open(device.devPath.c_str(), O_RDWR);
        if (_fd < 0) {
            int err = errno;
            if (err == EACCES) {
                return std::unexpected(std::format("permission denied opening '{}'"
                                                   "\n  try: sudo chmod 666 {}"
                                                   "\n  or add a udev rule:"
                                                   "\n    echo 'SUBSYSTEM==\"usb\", ATTR{{idVendor}}==\"{:04x}\", ATTR{{idProduct}}==\"{:04x}\", MODE=\"0666\"'"
                                                   "\n    | sudo tee /etc/udev/rules.d/99-usb-{:04x}-{:04x}.rules"
                                                   "\n    && sudo udevadm control --reload-rules && sudo udevadm trigger",
                    device.devPath, device.devPath, device.vendorId, device.productId, device.vendorId, device.productId));
            }
            return std::unexpected(std::format("failed to open '{}': {}", device.devPath, std::strerror(err)));
        }

        // detach kernel driver (e.g. dvb_usb_rtl28xxu) — ENODATA means none attached
        usbdevfs_disconnect_claim dc{};
        dc.interface = static_cast<unsigned>(interfaceNum);
        dc.flags     = USBDEVFS_DISCONNECT_CLAIM_EXCEPT_DRIVER;
        dc.driver[0] = '\0'; // disconnect any driver
        if (::ioctl(_fd, USBDEVFS_DISCONNECT_CLAIM, &dc) < 0) {
            // fallback: try separate disconnect + claim
            usbdevfs_ioctl cmd{};
            cmd.ifno          = interfaceNum;
            cmd.ioctl_code    = USBDEVFS_DISCONNECT;
            cmd.data          = nullptr;
            int disconnectRet = ::ioctl(_fd, USBDEVFS_IOCTL, &cmd);
            if (disconnectRet < 0 && errno != ENODATA) {
                // ENODATA = no driver attached — not an error
                int err = errno;
                ::close(_fd);
                _fd = -1;
                return std::unexpected(std::format("failed to detach kernel driver on interface {}: {}", interfaceNum, std::strerror(err)));
            }

            int iface = interfaceNum;
            if (::ioctl(_fd, USBDEVFS_CLAIMINTERFACE, &iface) < 0) {
                int err = errno;
                ::close(_fd);
                _fd = -1;
                if (err == EBUSY) {
                    return std::unexpected(std::format("interface {} already claimed on '{}'"
                                                       "\n  check: lsof {}",
                        interfaceNum, device.devPath, device.devPath));
                }
                return std::unexpected(std::format("failed to claim interface {}: {}", interfaceNum, std::strerror(err)));
            }
        }

        _interface = interfaceNum;
        return {};
    }

    void close() {
        if (_fd < 0) {
            return;
        }
        if (_interface >= 0) {
            int iface = _interface;
            ::ioctl(_fd, USBDEVFS_RELEASEINTERFACE, &iface);
            // re-attach kernel driver so the device returns to normal
            usbdevfs_ioctl cmd{};
            cmd.ifno       = _interface;
            cmd.ioctl_code = USBDEVFS_CONNECT;
            cmd.data       = nullptr;
            ::ioctl(_fd, USBDEVFS_IOCTL, &cmd);
            _interface = -1;
        }
        // closing the descriptor makes the kernel drop every queued transfer without writing to its buffer, so the
        // buffers are freed only after it
        ::close(_fd);
        _fd = -1;
        _queue.clear();
        _heldIndex  = kNoTransfer;
        _heldOffset = 0UZ;
    }

    // ── control transfers ───────────────────────────────────────────────────

    [[nodiscard]] Result controlOut(std::uint8_t bmRequestType, std::uint8_t bRequest, std::uint16_t wValue, std::uint16_t wIndex, std::span<const std::uint8_t> data, unsigned timeoutMs = 300) {
        usbdevfs_ctrltransfer ctrl{};
        ctrl.bRequestType = bmRequestType;
        ctrl.bRequest     = bRequest;
        ctrl.wValue       = wValue;
        ctrl.wIndex       = wIndex;
        ctrl.wLength      = static_cast<std::uint16_t>(data.size());
        ctrl.timeout      = timeoutMs;
        ctrl.data         = const_cast<std::uint8_t*>(data.data()); // kernel reads, does not write

        if (::ioctl(_fd, USBDEVFS_CONTROL, &ctrl) < 0) {
            return std::unexpected(formatTransferError("control OUT", errno));
        }
        return {};
    }

    [[nodiscard]] std::expected<std::vector<std::uint8_t>, std::string> controlIn(std::uint8_t bmRequestType, std::uint8_t bRequest, std::uint16_t wValue, std::uint16_t wIndex, std::uint16_t length, unsigned timeoutMs = 300) {
        std::vector<std::uint8_t> buf(std::max<std::uint16_t>(length, 8));

        usbdevfs_ctrltransfer ctrl{};
        ctrl.bRequestType = bmRequestType;
        ctrl.bRequest     = bRequest;
        ctrl.wValue       = wValue;
        ctrl.wIndex       = wIndex;
        ctrl.wLength      = static_cast<std::uint16_t>(buf.size());
        ctrl.timeout      = timeoutMs;
        ctrl.data         = buf.data();

        int ret = ::ioctl(_fd, USBDEVFS_CONTROL, &ctrl);
        if (ret < 0) {
            return std::unexpected(formatTransferError("control IN", errno));
        }
        buf.resize(static_cast<std::size_t>(ret));
        return buf;
    }

    // ── bulk transfers ──────────────────────────────────────────────────────

    [[nodiscard]] std::expected<std::size_t, std::string> bulkRead(std::uint8_t endpoint, std::span<std::uint8_t> buf, unsigned timeoutMs = 100) {
        usbdevfs_bulktransfer bulk{};
        bulk.ep      = endpoint;
        bulk.len     = static_cast<unsigned>(buf.size());
        bulk.timeout = timeoutMs;
        bulk.data    = buf.data();

        int ret = ::ioctl(_fd, USBDEVFS_BULK, &bulk);
        if (ret < 0) {
            if (errno == ETIMEDOUT) {
                return 0UZ; // timeout is not an error for bulk reads — just no data yet
            }
            return std::unexpected(formatTransferError("bulk read", errno));
        }
        return static_cast<std::size_t>(ret);
    }

    // Reads a bulk IN endpoint through a queue of nTransfers transfers of transferSize bytes each, all submitted to the
    // kernel at once. The device always has a buffer to fill while the caller works on the data of an earlier one;
    // between two synchronous bulkRead calls the endpoint has none, and a device with a small FIFO loses what it
    // samples in that gap. The first call submits the queue, and its endpoint and sizes hold until close().
    //
    // Each call hands out data of the oldest completed transfer, at most dst.size() bytes, and resubmits the transfer
    // once all of its data is handed out. It returns 0 when no transfer completes within timeoutMs. A transfer that
    // failed is resubmitted and its error returned.
    [[nodiscard]] std::expected<std::size_t, std::string> queuedBulkRead(std::uint8_t endpoint, std::span<std::uint8_t> dst, std::size_t nTransfers, std::size_t transferSize, unsigned timeoutMs = 100) {
        if (_fd < 0) {
            return std::unexpected(std::string("USBDevice: not open"));
        }
        if (_queue.empty()) {
            if (nTransfers == 0UZ || transferSize == 0UZ || transferSize > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                return std::unexpected(std::format("queued bulk read: {} transfers of {} bytes cannot be queued", nTransfers, transferSize));
            }
            _queue.resize(nTransfers);
            for (auto& transfer : _queue) {
                transfer.buffer.resize(transferSize);
                transfer.urb->endpoint = endpoint;
                if (auto submitted = submit(transfer); !submitted) {
                    return std::unexpected(submitted.error());
                }
            }
        }
        if (_heldIndex == kNoTransfer) {
            pollfd    ready{.fd = _fd, .events = POLLOUT, .revents = 0};
            const int nReady = ::poll(&ready, 1, static_cast<int>(timeoutMs));
            if (nReady < 0 && errno != EINTR) {
                return std::unexpected(formatTransferError("queued bulk read", errno));
            }
            if (nReady <= 0) {
                return 0UZ;
            }
            if ((ready.revents & POLLOUT) == 0) { // nothing completed: the device is gone or no transfer is queued
                return std::unexpected(std::string((ready.revents & POLLHUP) != 0 ? "queued bulk read: device disconnected" : "queued bulk read: no transfer queued"));
            }
            usbdevfs_urb* reaped = nullptr;
            if (::ioctl(_fd, USBDEVFS_REAPURBNDELAY, &reaped) < 0) {
                if (errno == EAGAIN) {
                    return 0UZ;
                }
                return std::unexpected(formatTransferError("queued bulk read", errno));
            }
            const auto found = std::ranges::find_if(_queue, [reaped](const QueuedTransfer& transfer) { return transfer.urb.get() == reaped; });
            if (found == _queue.end()) {
                return std::unexpected(std::string("queued bulk read: the kernel returned a transfer this queue did not submit"));
            }
            if (reaped->status != 0) {
                const int err = -reaped->status;
                std::ignore   = submit(*found);
                return std::unexpected(formatTransferError("queued bulk read", err));
            }
            _heldIndex  = static_cast<std::size_t>(found - _queue.begin());
            _heldOffset = 0UZ;
        }
        QueuedTransfer&   held   = _queue[_heldIndex];
        const std::size_t length = static_cast<std::size_t>(std::max(held.urb->actual_length, 0));
        const std::size_t nCopy  = std::min(length - std::min(_heldOffset, length), dst.size());
        std::memcpy(dst.data(), held.buffer.data() + _heldOffset, nCopy);
        _heldOffset += nCopy;
        if (_heldOffset >= length) {
            _heldIndex = kNoTransfer;
            if (auto submitted = submit(held); !submitted) {
                return std::unexpected(submitted.error());
            }
        }
        return nCopy;
    }

    [[nodiscard]] Result reset() {
        if (_fd < 0) {
            return std::unexpected(std::string("USBDevice: not open"));
        }
        if (::ioctl(_fd, USBDEVFS_RESET, nullptr) < 0) {
            return std::unexpected(std::format("USB reset failed: {}", std::strerror(errno)));
        }
        // after reset the interface claim is dropped — caller must re-claim + re-init
        _interface = -1;
        return {};
    }

    [[nodiscard]] Result clearHalt(std::uint8_t endpoint) {
        if (_fd < 0) {
            return std::unexpected(std::string("USBDevice: not open"));
        }
        unsigned ep = endpoint;
        if (::ioctl(_fd, USBDEVFS_CLEAR_HALT, &ep) < 0) {
            return std::unexpected(std::format("clear halt on endpoint 0x{:02X} failed: {}", endpoint, std::strerror(errno)));
        }
        return {};
    }

private:
    [[nodiscard]] Result submit(QueuedTransfer& transfer) {
        usbdevfs_urb& urb = *transfer.urb;
        urb.type          = USBDEVFS_URB_TYPE_BULK;
        urb.status        = 0;
        urb.flags         = 0U;
        urb.buffer        = transfer.buffer.data();
        urb.buffer_length = static_cast<int>(transfer.buffer.size());
        urb.actual_length = 0;
        if (::ioctl(_fd, USBDEVFS_SUBMITURB, &urb) < 0) {
            return std::unexpected(formatTransferError("bulk read submit", errno));
        }
        return {};
    }

    [[nodiscard]] static std::string formatTransferError(std::string_view op, int err) {
        std::string_view hint;
        switch (err) {
        case ENODEV: hint = " (device disconnected)"; break;
        case EPIPE: hint = " (transfer stalled — device rejected request)"; break;
        case ETIMEDOUT: hint = " (transfer timed out)"; break;
        case EBUSY: hint = " (endpoint busy)"; break;
        case EIO: hint = " (I/O error — possible hardware fault)"; break;
        default: hint = ""; break;
        }
        return std::format("{} failed: {}{}", op, std::strerror(err), hint);
    }
};

#else // !__linux__

// stub: non-Linux platforms use WebUSB shims or similar; permissions handled by the browser/OS
[[nodiscard]] inline bool         canAccessUSBDevice(const USBDeviceInfo&) { return true; }
inline std::vector<USBDeviceInfo> enumerateUSBDevices(std::span<const USBDeviceId> = {}) { return {}; }

struct USBDevice {
    using Result = std::expected<void, std::string>;

    [[nodiscard]] bool   isOpen() const { return false; }
    [[nodiscard]] Result open(const USBDeviceInfo&, int = 0) { return std::unexpected(std::string("USBDevice: Linux-only")); }
    void                 close() {}

    [[nodiscard]] Result controlOut(std::uint8_t, std::uint8_t, std::uint16_t, std::uint16_t, std::span<const std::uint8_t>, unsigned = 300) { return std::unexpected(std::string("USBDevice: Linux-only")); }

    [[nodiscard]] std::expected<std::vector<std::uint8_t>, std::string> controlIn(std::uint8_t, std::uint8_t, std::uint16_t, std::uint16_t, std::uint16_t, unsigned = 300) { return std::unexpected(std::string("USBDevice: Linux-only")); }

    [[nodiscard]] std::expected<std::size_t, std::string> bulkRead(std::uint8_t, std::span<std::uint8_t>, unsigned = 100) { return std::unexpected(std::string("USBDevice: Linux-only")); }

    [[nodiscard]] std::expected<std::size_t, std::string> queuedBulkRead(std::uint8_t, std::span<std::uint8_t>, std::size_t, std::size_t, unsigned = 100) { return std::unexpected(std::string("USBDevice: Linux-only")); }

    [[nodiscard]] Result reset() { return std::unexpected(std::string("USBDevice: Linux-only")); }
    [[nodiscard]] Result clearHalt(std::uint8_t) { return std::unexpected(std::string("USBDevice: Linux-only")); }
};

#endif // __linux__

} // namespace gr::blocks::common

#endif // GNURADIO_USB_DEVICE_HPP
