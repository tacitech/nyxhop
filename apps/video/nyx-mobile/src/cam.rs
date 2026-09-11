//! The phone's own camera as the aircraft video source (Android only): camera2 through the
//! NDK (libcamera2ndk + AImageReader), no Java. Frames arrive as YUV_420_888, are converted
//! to RGB and land in the shared webcam slot the transmit worker reads (`SourceKind::Webcam`,
//! shown as "Phone camera" on the phone), so the encode/send path is exactly the PC one.
//!
//! The thread opens the back camera only while the worker wants it (`wanted`), at the
//! smallest YUV mode covering the size the host asked for (`want_wh`), and turns the picture
//! by 180 degrees when the phone is held the other way round (sensor orientation against
//! the display rotation). The camera permission is asked for here; the answer shows up in
//! `checkSelfPermission` (no Java callback without Java code).

use std::ffi::{c_int, c_void, CStr};
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use ndk_sys as n;
use nyx_common::logging::log;
use nyx_common::source::{yuv420_to_rgb, WebcamShared, Yuv420};

// ndk-sys links libmediandk (feature "media") but not the camera library.
#[link(name = "camera2ndk")]
unsafe extern "C" {}

/// Set by the device callbacks (the system took the camera, or it failed): reopen.
static LOST: AtomicBool = AtomicBool::new(false);

unsafe extern "C" fn on_disconnected(_: *mut c_void, _: *mut n::ACameraDevice) {
    log("phonecam: disconnected by the system");
    LOST.store(true, Ordering::Relaxed);
}

unsafe extern "C" fn on_error(_: *mut c_void, _: *mut n::ACameraDevice, err: c_int) {
    log(&format!("phonecam: device error {err}"));
    LOST.store(true, Ordering::Relaxed);
}

unsafe extern "C" fn on_session(_: *mut c_void, _: *mut n::ACameraCaptureSession) {}

pub fn spawn(shared: Arc<WebcamShared>) {
    std::thread::Builder::new()
        .name("phonecam".into())
        .spawn(move || run(shared))
        .expect("spawn phonecam");
}

fn set_status(shared: &WebcamShared, s: String) {
    log(&format!("phonecam: {s}"));
    *shared.status.lock().unwrap() = s;
}

/// An open camera streaming into an image reader. Everything is torn down in `Drop`, in
/// the reverse order of creation.
struct Cam {
    mgr: *mut n::ACameraManager,
    dev: *mut n::ACameraDevice,
    reader: *mut n::AImageReader,
    container: *mut n::ACaptureSessionOutputContainer,
    output: *mut n::ACaptureSessionOutput,
    target: *mut n::ACameraOutputTarget,
    req: *mut n::ACaptureRequest,
    session: *mut n::ACameraCaptureSession,
    // the framework copies these, but keeping them alive costs nothing
    _dev_cbs: Box<n::ACameraDevice_StateCallbacks>,
    _sess_cbs: Box<n::ACameraCaptureSession_stateCallbacks>,
    w: usize,
    h: usize,
    /// degrees the sensor picture must be turned clockwise to be upright in portrait
    sensor_orientation: i32,
    /// the size the host asked for when this camera was opened (reopen when it changes)
    opened_for: Option<(usize, usize)>,
}

unsafe impl Send for Cam {}

impl Drop for Cam {
    fn drop(&mut self) {
        unsafe {
            if !self.session.is_null() {
                n::ACameraCaptureSession_stopRepeating(self.session);
                n::ACameraCaptureSession_close(self.session);
            }
            if !self.req.is_null() {
                n::ACaptureRequest_free(self.req);
            }
            if !self.target.is_null() {
                n::ACameraOutputTarget_free(self.target);
            }
            if !self.output.is_null() {
                n::ACaptureSessionOutput_free(self.output);
            }
            if !self.container.is_null() {
                n::ACaptureSessionOutputContainer_free(self.container);
            }
            if !self.dev.is_null() {
                n::ACameraDevice_close(self.dev);
            }
            if !self.reader.is_null() {
                n::AImageReader_delete(self.reader);
            }
            if !self.mgr.is_null() {
                n::ACameraManager_delete(self.mgr);
            }
        }
    }
}

/// What the characteristics say about one camera.
struct Info {
    id: std::ffi::CString,
    facing_back: bool,
    orientation: i32,
    /// YUV_420_888 output sizes
    sizes: Vec<(usize, usize)>,
}

unsafe fn read_info(mgr: *mut n::ACameraManager, id: *const std::os::raw::c_char) -> Option<Info> {
    let mut meta: *mut n::ACameraMetadata = ptr::null_mut();
    if n::ACameraManager_getCameraCharacteristics(mgr, id, &mut meta) != n::camera_status_t::ACAMERA_OK || meta.is_null() {
        return None;
    }
    let entry = |tag: n::acamera_metadata_tag| -> Option<n::ACameraMetadata_const_entry> {
        let mut e: n::ACameraMetadata_const_entry = std::mem::zeroed();
        (n::ACameraMetadata_getConstEntry(meta, tag.0, &mut e) == n::camera_status_t::ACAMERA_OK && e.count > 0).then_some(e)
    };
    let facing_back = entry(n::acamera_metadata_tag::ACAMERA_LENS_FACING)
        .map(|e| u32::from(*e.data.u8_) == n::acamera_metadata_enum_acamera_lens_facing::ACAMERA_LENS_FACING_BACK.0)
        .unwrap_or(false);
    let orientation = entry(n::acamera_metadata_tag::ACAMERA_SENSOR_ORIENTATION).map(|e| *e.data.i32_).unwrap_or(90);
    let mut sizes = Vec::new();
    if let Some(e) = entry(n::acamera_metadata_tag::ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS) {
        // quadruples: format, width, height, input(1)/output(0)
        let v = std::slice::from_raw_parts(e.data.i32_, e.count as usize);
        for q in v.chunks_exact(4) {
            if q[0] == n::AIMAGE_FORMATS::AIMAGE_FORMAT_YUV_420_888.0 as i32 && q[3] == 0 && q[1] > 0 && q[2] > 0 {
                sizes.push((q[1] as usize, q[2] as usize));
            }
        }
    }
    n::ACameraMetadata_free(meta);
    Some(Info { id: CStr::from_ptr(id).to_owned(), facing_back, orientation, sizes })
}

/// The smallest mode that covers the wanted size; the largest one when none does.
fn pick_size(sizes: &[(usize, usize)], want: (usize, usize)) -> Option<(usize, usize)> {
    let covering = sizes.iter().copied().filter(|&(w, h)| w >= want.0 && h >= want.1).min_by_key(|&(w, h)| w * h);
    covering.or_else(|| sizes.iter().copied().max_by_key(|&(w, h)| w * h))
}

impl Cam {
    unsafe fn open(want: (usize, usize)) -> Result<Cam, String> {
        let mgr = n::ACameraManager_create();
        if mgr.is_null() {
            return Err("no camera manager".into());
        }
        let mut cam = Cam {
            mgr,
            dev: ptr::null_mut(),
            reader: ptr::null_mut(),
            container: ptr::null_mut(),
            output: ptr::null_mut(),
            target: ptr::null_mut(),
            req: ptr::null_mut(),
            session: ptr::null_mut(),
            _dev_cbs: Box::new(n::ACameraDevice_StateCallbacks { context: ptr::null_mut(), onDisconnected: Some(on_disconnected), onError: Some(on_error) }),
            _sess_cbs: Box::new(n::ACameraCaptureSession_stateCallbacks { context: ptr::null_mut(), onClosed: Some(on_session), onReady: Some(on_session), onActive: Some(on_session) }),
            w: 0,
            h: 0,
            sensor_orientation: 90,
            opened_for: Some(want),
        };
        // which camera: the back one, else the first
        let mut ids: *mut n::ACameraIdList = ptr::null_mut();
        let st = n::ACameraManager_getCameraIdList(mgr, &mut ids);
        if st != n::camera_status_t::ACAMERA_OK || ids.is_null() {
            return Err(format!("camera list: status {}", st.0));
        }
        let mut infos = Vec::new();
        for i in 0..(*ids).numCameras.max(0) as usize {
            let id = *(*ids).cameraIds.add(i);
            if let Some(info) = read_info(mgr, id) {
                infos.push(info);
            }
        }
        n::ACameraManager_deleteCameraIdList(ids);
        let info = infos.iter().find(|i| i.facing_back && !i.sizes.is_empty()).or_else(|| infos.iter().find(|i| !i.sizes.is_empty()));
        let Some(info) = info else {
            return Err("no camera with a YUV output".into());
        };
        let (w, h) = pick_size(&info.sizes, want).ok_or("no size")?;
        cam.w = w;
        cam.h = h;
        cam.sensor_orientation = info.orientation;
        // open the device
        let st = n::ACameraManager_openCamera(mgr, info.id.as_ptr(), &mut *cam._dev_cbs, &mut cam.dev);
        if st != n::camera_status_t::ACAMERA_OK || cam.dev.is_null() {
            return Err(format!("open {}: status {}", info.id.to_string_lossy(), st.0));
        }
        // the reader the frames land in (4 buffers: one being read, others in flight)
        let st = n::AImageReader_new(w as i32, h as i32, n::AIMAGE_FORMATS::AIMAGE_FORMAT_YUV_420_888.0 as i32, 4, &mut cam.reader);
        if st != n::media_status_t::AMEDIA_OK || cam.reader.is_null() {
            return Err(format!("image reader {w}x{h}: status {}", st.0));
        }
        let mut window: *mut n::ANativeWindow = ptr::null_mut();
        if n::AImageReader_getWindow(cam.reader, &mut window) != n::media_status_t::AMEDIA_OK || window.is_null() {
            return Err("reader window".into());
        }
        // session output = that window
        if n::ACaptureSessionOutputContainer_create(&mut cam.container) != n::camera_status_t::ACAMERA_OK {
            return Err("output container".into());
        }
        if n::ACaptureSessionOutput_create(window, &mut cam.output) != n::camera_status_t::ACAMERA_OK {
            return Err("session output".into());
        }
        if n::ACaptureSessionOutputContainer_add(cam.container, cam.output) != n::camera_status_t::ACAMERA_OK {
            return Err("container add".into());
        }
        // the repeating preview request, targeting the window
        if n::ACameraDevice_createCaptureRequest(cam.dev, n::ACameraDevice_request_template::TEMPLATE_PREVIEW, &mut cam.req) != n::camera_status_t::ACAMERA_OK {
            return Err("capture request".into());
        }
        if n::ACameraOutputTarget_create(window, &mut cam.target) != n::camera_status_t::ACAMERA_OK {
            return Err("output target".into());
        }
        if n::ACaptureRequest_addTarget(cam.req, cam.target) != n::camera_status_t::ACAMERA_OK {
            return Err("add target".into());
        }
        // a steady 30 fps when the camera offers it (a range the camera lacks is refused
        // quietly and the preview template's choice stands)
        let fps: [i32; 2] = [30, 30];
        let _ = n::ACaptureRequest_setEntry_i32(cam.req, n::acamera_metadata_tag::ACAMERA_CONTROL_AE_TARGET_FPS_RANGE.0, 2, fps.as_ptr());
        let st = n::ACameraDevice_createCaptureSession(cam.dev, cam.container, &*cam._sess_cbs, &mut cam.session);
        if st != n::camera_status_t::ACAMERA_OK || cam.session.is_null() {
            return Err(format!("capture session: status {}", st.0));
        }
        let st = n::ACameraCaptureSession_setRepeatingRequest(cam.session, ptr::null_mut(), 1, &mut cam.req, ptr::null_mut());
        if st != n::camera_status_t::ACAMERA_OK {
            return Err(format!("repeating request: status {}", st.0));
        }
        Ok(cam)
    }

    /// The newest frame, converted; None when nothing new has arrived.
    unsafe fn grab(&mut self, rotate180: bool) -> Option<nyx_common::RgbFrame> {
        let mut img: *mut n::AImage = ptr::null_mut();
        if n::AImageReader_acquireLatestImage(self.reader, &mut img) != n::media_status_t::AMEDIA_OK || img.is_null() {
            return None;
        }
        let (mut w, mut h) = (0i32, 0i32);
        n::AImage_getWidth(img, &mut w);
        n::AImage_getHeight(img, &mut h);
        let mut plane = |i: c_int| -> Option<(&[u8], usize, usize)> {
            let (mut data, mut len, mut rs, mut ps) = (ptr::null_mut::<u8>(), 0i32, 0i32, 0i32);
            if n::AImage_getPlaneData(img, i, &mut data, &mut len) != n::media_status_t::AMEDIA_OK || data.is_null() || len <= 0 {
                return None;
            }
            n::AImage_getPlaneRowStride(img, i, &mut rs);
            n::AImage_getPlanePixelStride(img, i, &mut ps);
            Some((std::slice::from_raw_parts(data as *const u8, len as usize), rs.max(0) as usize, ps.max(1) as usize))
        };
        let out = match (plane(0), plane(1), plane(2)) {
            (Some((y, ys, _)), Some((u, us, ups)), Some((v, _, _))) if w > 0 && h > 0 => {
                let (w, h) = (w as usize, h as usize);
                // the planes may be shorter than stride*rows by the padding of the last row:
                // stay inside what the image says it holds
                let ok = y.len() >= ys * (h - 1) + w && u.len() >= us * (h / 2 - 1) + (w / 2 - 1) * ups + 1 && v.len() >= us * (h / 2 - 1) + (w / 2 - 1) * ups + 1;
                ok.then(|| yuv420_to_rgb(&Yuv420 { y, y_stride: ys, u, v, uv_stride: us, uv_pixel_stride: ups }, w, h, rotate180))
            }
            _ => None,
        };
        n::AImage_delete(img);
        out
    }
}

fn run(shared: Arc<WebcamShared>) {
    let mut cam: Option<Cam> = None;
    let mut asked: Option<Instant> = None;
    let mut rotate180 = false;
    let mut rot_checked = Instant::now() - Duration::from_secs(10);
    let mut cnt = 0u64;
    let mut conv_us = 0u64;
    let mut report = Instant::now();
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        if !shared.wanted.load(Ordering::Relaxed) {
            if cam.take().is_some() {
                set_status(&shared, "idle".into());
            }
            std::thread::sleep(Duration::from_millis(150));
            continue;
        }
        let want = shared.want().unwrap_or((640, 480));
        if let Some(c) = &cam {
            if LOST.swap(false, Ordering::Relaxed) {
                set_status(&shared, "camera lost, reopening".into());
                cam = None;
                std::thread::sleep(Duration::from_millis(500));
                continue;
            }
            if c.opened_for != Some(want) {
                set_status(&shared, format!("size wanted {}x{}: reopening", want.0, want.1));
                cam = None;
                continue;
            }
        }
        if cam.is_none() {
            match crate::jni_ctx::camera_permitted() {
                Some(true) => {}
                Some(false) => {
                    // ask, then wait; ask again after a while (the dialog may have been dismissed)
                    if asked.is_none_or(|t| t.elapsed() > Duration::from_secs(20)) {
                        asked = Some(Instant::now());
                        crate::jni_ctx::request_camera();
                        set_status(&shared, "waiting for the camera permission (allow it in the dialog)".into());
                    }
                    std::thread::sleep(Duration::from_millis(500));
                    continue;
                }
                None => {
                    set_status(&shared, "no activity to ask for the camera".into());
                    std::thread::sleep(Duration::from_secs(1));
                    continue;
                }
            }
            LOST.store(false, Ordering::Relaxed);
            match unsafe { Cam::open(want) } {
                Ok(c) => {
                    set_status(&shared, format!("open {}x{} (sensor {} deg)", c.w, c.h, c.sensor_orientation));
                    cam = Some(c);
                    cnt = 0;
                    conv_us = 0;
                    report = Instant::now();
                }
                Err(e) => {
                    set_status(&shared, format!("open error: {e}"));
                    cam = None;
                    std::thread::sleep(Duration::from_secs(2));
                }
            }
            continue;
        }
        let c = cam.as_mut().unwrap();
        // which way round is the landscape: once a second is plenty
        if rot_checked.elapsed() > Duration::from_secs(1) {
            rot_checked = Instant::now();
            if let Some(rot) = crate::jni_ctx::display_rotation() {
                let need = (c.sensor_orientation - rot).rem_euclid(360);
                let flip = need == 180;
                if flip != rotate180 {
                    log(&format!("phonecam: display rotation {rot}, sensor {}: turning {}", c.sensor_orientation, if flip { "180" } else { "0" }));
                    rotate180 = flip;
                }
            }
        }
        let t0 = Instant::now();
        match unsafe { c.grab(rotate180) } {
            Some(frame) => {
                conv_us += t0.elapsed().as_micros() as u64;
                cnt += 1;
                let (w, h) = (frame.width, frame.height);
                *shared.frame.lock().unwrap() = Some(frame);
                let el = report.elapsed().as_secs_f32();
                if el >= 2.0 {
                    set_status(&shared, format!("cap {:.0}fps {w}x{h} convert {}ms", cnt as f32 / el, conv_us / 1000 / cnt.max(1)));
                    cnt = 0;
                    conv_us = 0;
                    report = Instant::now();
                }
            }
            None => std::thread::sleep(Duration::from_millis(3)),
        }
    }
}
