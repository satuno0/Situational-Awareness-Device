#include "desktop_camera.hpp"

#include <cstdint>
#include <limits>
#include <sstream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <wrl/client.h>
#endif

namespace sa {

struct DesktopCamera::Impl
{
#ifdef _WIN32
    Microsoft::WRL::ComPtr<IMFSourceReader> reader;
    Microsoft::WRL::ComPtr<IMFMediaSource> source;
    UINT32 width = 0;
    UINT32 height = 0;
    LONG stride = 0;
    bool media_foundation_started = false;
#endif
};

namespace {

#ifdef _WIN32
std::string hresultText(const char* operation, HRESULT result)
{
    std::ostringstream message;
    message << operation << " failed (HRESULT 0x" << std::hex
            << static_cast<unsigned long>(result) << ")";
    return message.str();
}
#endif

} // namespace

DesktopCamera::DesktopCamera() = default;
DesktopCamera::~DesktopCamera() { close(); }

bool DesktopCamera::open(std::string& error)
{
    close();
#ifndef _WIN32
    error = "desktop webcam capture is available only on Windows";
    return false;
#else
    impl_ = std::make_unique<Impl>();
    auto fail = [&](const std::string& message) {
        error = message;
        close();
        return false;
    };

    HRESULT result = MFStartup(MF_VERSION, MFSTARTUP_FULL);
    if (FAILED(result)) return fail(hresultText("Media Foundation startup", result));
    impl_->media_foundation_started = true;

    Microsoft::WRL::ComPtr<IMFAttributes> attributes;
    result = MFCreateAttributes(attributes.GetAddressOf(), 1);
    if (FAILED(result)) return fail(hresultText("MFCreateAttributes", result));
    result = attributes->SetGUID(
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if (FAILED(result)) return fail(hresultText("Set webcam device type", result));

    IMFActivate** devices = nullptr;
    UINT32 device_count = 0;
    result = MFEnumDeviceSources(attributes.Get(), &devices, &device_count);
    if (FAILED(result)) return fail(hresultText("Enumerate webcams", result));
    if (device_count == 0)
    {
        CoTaskMemFree(devices);
        return fail("no webcam was found");
    }

    result = devices[0]->ActivateObject(IID_PPV_ARGS(impl_->source.GetAddressOf()));
    for (UINT32 i = 0; i < device_count; ++i) devices[i]->Release();
    CoTaskMemFree(devices);
    if (FAILED(result)) return fail(hresultText("Open the default webcam", result));

    Microsoft::WRL::ComPtr<IMFAttributes> reader_attributes;
    result = MFCreateAttributes(reader_attributes.GetAddressOf(), 1);
    if (FAILED(result)) return fail(hresultText("Create webcam reader settings", result));
    result = reader_attributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE);
    if (FAILED(result)) return fail(hresultText("Enable webcam video processing", result));
    result = MFCreateSourceReaderFromMediaSource(
        impl_->source.Get(), reader_attributes.Get(), impl_->reader.GetAddressOf());
    if (FAILED(result)) return fail(hresultText("Create webcam frame reader", result));

    Microsoft::WRL::ComPtr<IMFMediaType> output_type;
    result = MFCreateMediaType(output_type.GetAddressOf());
    if (FAILED(result)) return fail(hresultText("Create webcam output format", result));
    result = output_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    if (FAILED(result)) return fail(hresultText("Set webcam output type", result));
    result = output_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
    if (FAILED(result)) return fail(hresultText("Set webcam RGB format", result));
    result = impl_->reader->SetCurrentMediaType(
        static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, output_type.Get());
    if (FAILED(result)) return fail(hresultText("Configure webcam RGB output", result));

    Microsoft::WRL::ComPtr<IMFMediaType> current_type;
    result = impl_->reader->GetCurrentMediaType(
        static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), current_type.GetAddressOf());
    if (FAILED(result)) return fail(hresultText("Read webcam output format", result));
    result = MFGetAttributeSize(current_type.Get(), MF_MT_FRAME_SIZE,
                                &impl_->width, &impl_->height);
    if (FAILED(result) || impl_->width == 0 || impl_->height == 0 ||
        impl_->width > static_cast<UINT32>(std::numeric_limits<int>::max() / 4) ||
        impl_->height > static_cast<UINT32>(std::numeric_limits<int>::max()))
        return fail("webcam returned invalid frame dimensions");

    UINT32 stride_value = 0;
    result = current_type->GetUINT32(MF_MT_DEFAULT_STRIDE, &stride_value);
    if (SUCCEEDED(result))
    {
        impl_->stride = static_cast<LONG>(stride_value);
    }
    else
    {
        result = MFGetStrideForBitmapInfoHeader(
            MFVideoFormat_RGB32.Data1, impl_->width, &impl_->stride);
        if (FAILED(result)) return fail(hresultText("Determine webcam frame stride", result));
    }
    if (impl_->stride == 0)
        return fail("webcam returned an invalid frame stride");

    return true;
#endif
}

bool DesktopCamera::readRgb(std::vector<uint8_t>& pixels, int& width, int& height,
                            std::string& error)
{
#ifndef _WIN32
    (void)pixels;
    (void)width;
    (void)height;
    error = "desktop webcam capture is available only on Windows";
    return false;
#else
    if (!impl_ || !impl_->reader)
    {
        error = "webcam is not open";
        return false;
    }

    Microsoft::WRL::ComPtr<IMFSample> sample;
    DWORD flags = 0;
    HRESULT result = S_OK;
    do
    {
        flags = 0;
        result = impl_->reader->ReadSample(
            static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), 0, nullptr, &flags,
            nullptr, sample.ReleaseAndGetAddressOf());
        if (FAILED(result))
        {
            error = hresultText("Read webcam frame", result);
            return false;
        }
        if ((flags & MF_SOURCE_READERF_ENDOFSTREAM) != 0)
        {
            error = "webcam stream ended";
            return false;
        }
        if ((flags & MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED) != 0)
        {
            error = "webcam output format changed unexpectedly";
            return false;
        }
    } while (!sample && (flags & MF_SOURCE_READERF_STREAMTICK) != 0);

    if (!sample)
    {
        error = "webcam returned no frame";
        return false;
    }

    Microsoft::WRL::ComPtr<IMFMediaBuffer> buffer;
    result = sample->ConvertToContiguousBuffer(buffer.GetAddressOf());
    if (FAILED(result))
    {
        error = hresultText("Get webcam frame buffer", result);
        return false;
    }

    BYTE* source_pixels = nullptr;
    DWORD maximum_length = 0;
    DWORD current_length = 0;
    result = buffer->Lock(&source_pixels, &maximum_length, &current_length);
    if (FAILED(result))
    {
        error = hresultText("Lock webcam frame buffer", result);
        return false;
    }

    const std::size_t row_stride = static_cast<std::size_t>(
        impl_->stride < 0 ? -static_cast<int64_t>(impl_->stride) : impl_->stride);
    const std::size_t source_row_bytes = static_cast<std::size_t>(impl_->width) * 4u;
    const std::size_t required_bytes = row_stride * static_cast<std::size_t>(impl_->height);
    if (!source_pixels || row_stride < source_row_bytes || current_length > maximum_length ||
        current_length < required_bytes)
    {
        buffer->Unlock();
        error = "webcam frame buffer is smaller than its declared format";
        return false;
    }

    width = static_cast<int>(impl_->width);
    height = static_cast<int>(impl_->height);
    pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3u);
    for (int y = 0; y < height; ++y)
    {
        const int source_y = impl_->stride < 0 ? height - 1 - y : y;
        const BYTE* source_row = source_pixels +
            static_cast<std::size_t>(source_y) * row_stride;
        uint8_t* output_row = pixels.data() +
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 3u;
        for (int x = 0; x < width; ++x)
        {
            const std::size_t source_index = static_cast<std::size_t>(x) * 4u;
            const std::size_t output_index = static_cast<std::size_t>(x) * 3u;
            output_row[output_index] = source_row[source_index + 2];
            output_row[output_index + 1] = source_row[source_index + 1];
            output_row[output_index + 2] = source_row[source_index];
        }
    }

    result = buffer->Unlock();
    if (FAILED(result))
    {
        error = hresultText("Unlock webcam frame buffer", result);
        return false;
    }
    return true;
#endif
}

void DesktopCamera::close()
{
#ifdef _WIN32
    if (!impl_) return;
    impl_->reader.Reset();
    if (impl_->source) impl_->source->Shutdown();
    impl_->source.Reset();
    if (impl_->media_foundation_started) MFShutdown();
#endif
    impl_.reset();
}

} // namespace sa
