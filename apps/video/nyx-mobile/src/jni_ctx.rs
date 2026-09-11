//! The few things the app needs from the Java side (Android only), all through JNI on the
//! NativeActivity: the network binding, the camera permission, the display rotation.
//! There is no Java code in the app; every call goes through the Activity object.

use std::sync::atomic::{AtomicPtr, Ordering};

use jni::objects::{JObject, JValue};
use jni::JNIEnv;

static VM: AtomicPtr<std::ffi::c_void> = AtomicPtr::new(std::ptr::null_mut());
static ACTIVITY: AtomicPtr<std::ffi::c_void> = AtomicPtr::new(std::ptr::null_mut());

pub fn remember(vm: *mut std::ffi::c_void, activity: *mut std::ffi::c_void) {
    VM.store(vm, Ordering::Relaxed);
    ACTIVITY.store(activity, Ordering::Relaxed);
}

/// Run `f` with a JNI env attached to this thread and the Activity. None before
/// `remember`, or when JNI fails.
pub fn with_env<R>(f: impl FnOnce(&mut JNIEnv, &JObject) -> jni::errors::Result<R>) -> Option<R> {
    let vmp = VM.load(Ordering::Relaxed);
    let actp = ACTIVITY.load(Ordering::Relaxed);
    if vmp.is_null() || actp.is_null() {
        return None;
    }
    let vm = unsafe { jni::JavaVM::from_raw(vmp.cast()) }.ok()?;
    let mut env = vm.attach_current_thread().ok()?;
    let activity = unsafe { JObject::from_raw(actp.cast()) };
    match f(&mut env, &activity) {
        Ok(r) => Some(r),
        Err(e) => {
            log::warn!("jni: {e}");
            // a pending Java exception would poison the next call on this thread
            let _ = env.exception_clear();
            None
        }
    }
}

/// Android hands every app socket to the DEFAULT network, which is WiFi whenever
/// WiFi has internet and the wired link does not. The board hangs off USB-C
/// Ethernet, so on a phone with both up the app tried to reach 192.168.0.10 over
/// WiFi and failed, even though eth0 pinged the board in 0.8 ms. Routing cannot
/// fix it from our side: the per-uid rules key off the socket fwmark, not the
/// source address, and SO_BINDTODEVICE needs CAP_NET_RAW. The supported way is
/// ConnectivityManager.bindProcessToNetwork() on the Ethernet network. Called
/// before every connect so plugging the cable in later also works.
/// Some(true) = bound to a wired network now.
pub fn bind_wired() -> Option<bool> {
    with_env(|env, activity| {
        use jni::objects::JObjectArray;
        let name = env.new_string("connectivity")?;
        let cm = env
            .call_method(activity, "getSystemService", "(Ljava/lang/String;)Ljava/lang/Object;", &[JValue::Object(&name)])?
            .l()?;
        let nets = env.call_method(&cm, "getAllNetworks", "()[Landroid/net/Network;", &[])?.l()?;
        let nets = JObjectArray::from(nets);
        let n = env.get_array_length(&nets)?;
        for i in 0..n {
            let net = env.get_object_array_element(&nets, i)?;
            let caps = env
                .call_method(&cm, "getNetworkCapabilities", "(Landroid/net/Network;)Landroid/net/NetworkCapabilities;", &[JValue::Object(&net)])?
                .l()?;
            if caps.is_null() {
                continue;
            }
            // NetworkCapabilities.TRANSPORT_ETHERNET = 3
            if env.call_method(&caps, "hasTransport", "(I)Z", &[JValue::Int(3)])?.z()? {
                return env
                    .call_method(&cm, "bindProcessToNetwork", "(Landroid/net/Network;)Z", &[JValue::Object(&net)])?
                    .z();
            }
        }
        Ok(false)
    })
}

const CAMERA: &str = "android.permission.CAMERA";

/// Some(true) = the app may use the camera.
pub fn camera_permitted() -> Option<bool> {
    with_env(|env, activity| {
        let perm = env.new_string(CAMERA)?;
        // PackageManager.PERMISSION_GRANTED = 0
        Ok(env.call_method(activity, "checkSelfPermission", "(Ljava/lang/String;)I", &[JValue::Object(&perm)])?.i()? == 0)
    })
}

/// Show the system's permission dialog for the camera (the answer shows up in
/// `camera_permitted` a moment later; there is no Java callback to receive it).
pub fn request_camera() -> bool {
    with_env(|env, activity| {
        let perm = env.new_string(CAMERA)?;
        let arr = env.new_object_array(1, "java/lang/String", &perm)?;
        env.call_method(activity, "requestPermissions", "([Ljava/lang/String;I)V", &[JValue::Object(&arr), JValue::Int(1)])?;
        Ok(())
    })
    .is_some()
}

/// The screen's rotation from the phone's natural orientation, in degrees (0, 90, 180,
/// 270): tells which way round the landscape is, so the camera picture can be turned.
pub fn display_rotation() -> Option<i32> {
    with_env(|env, activity| {
        let disp = env.call_method(activity, "getDisplay", "()Landroid/view/Display;", &[])?.l()?;
        if disp.is_null() {
            return Ok(0);
        }
        // Surface.ROTATION_0/90/180/270 = 0..3
        Ok(env.call_method(&disp, "getRotation", "()I", &[])?.i()? * 90)
    })
}
