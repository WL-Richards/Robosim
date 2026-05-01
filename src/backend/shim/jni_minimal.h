#pragma once

#include <cstdint>

namespace robosim::backend::shim::jni {

using jboolean = unsigned char;
using jbyte = signed char;
using jshort = short;
using jint = int;
using jlong = long long;
using jfloat = float;
using jsize = jint;

struct _jobject {};
struct _jclass : _jobject {};
struct _jstring : _jobject {};
struct _jfloatArray : _jobject {};
struct _jintArray : _jobject {};
struct _jshortArray : _jobject {};
struct _jmethodID {};

using jobject = _jobject*;
using jclass = _jclass*;
using jstring = _jstring*;
using jfloatArray = _jfloatArray*;
using jintArray = _jintArray*;
using jshortArray = _jshortArray*;
using jmethodID = _jmethodID*;

struct JNIEnv;

/**
 * Minimal JNI function-table prefix used by the repo-owned HAL JNI adapter.
 *
 * The reserved slot counts pin the standard JNI table indices for the few
 * functions Cycle 52 needs, without making the simulator build depend on a
 * host JDK include path.
 */
struct JNINativeInterface {
  void* reserved_before_get_object_class[31]{};
  jclass (*GetObjectClass)(JNIEnv*, jobject) = nullptr;
  void* reserved_before_get_method_id[1]{};
  jmethodID (*GetMethodID)(JNIEnv*, jclass, const char*, const char*) = nullptr;
  void* reserved_before_call_void_method[27]{};
  void (*CallVoidMethod)(JNIEnv*, jobject, jmethodID, ...) = nullptr;
  void* reserved_before_new_string_utf[105]{};
  jstring (*NewStringUTF)(JNIEnv*, const char*) = nullptr;
  void* reserved_before_get_string_utf_chars[1]{};
  const char* (*GetStringUTFChars)(JNIEnv*, jstring, jboolean*) = nullptr;
  void (*ReleaseStringUTFChars)(JNIEnv*, jstring, const char*) = nullptr;
  void* reserved_before_set_short_array_region[39]{};
  void (*SetShortArrayRegion)(JNIEnv*, jshortArray, jsize, jsize, const jshort*) = nullptr;
  void (*SetIntArrayRegion)(JNIEnv*, jintArray, jsize, jsize, const jint*) = nullptr;
  void* SetLongArrayRegion = nullptr;
  void (*SetFloatArrayRegion)(JNIEnv*, jfloatArray, jsize, jsize, const jfloat*) = nullptr;
  void* SetDoubleArrayRegion = nullptr;
  void* reserved_before_get_direct_buffer_address[15]{};
  void* (*GetDirectBufferAddress)(JNIEnv*, jobject) = nullptr;
};

struct JNIEnv {
  const JNINativeInterface* functions = nullptr;
};

}  // namespace robosim::backend::shim::jni
