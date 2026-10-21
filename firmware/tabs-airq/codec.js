/*
 * ChirpStack v4 payload codec for the tabs-airq air quality sensor (tabsfw firmware).
 *
 * Uplinks:
 *   103  status (8 bytes)
 *   204  configuration report (tag-length-value)
 *   201  fragmentation answers (frag service, FUOTA)
 *   203  firmware management answers (fwman service, FUOTA)
 *   0    MAC commands only -- no application data
 *
 * Downlinks (encodeDownlink, port 204): configuration, see encodeConfig().
 *
 * Status uplink, port 103 (multi-byte fields little-endian):
 *   [0]     bit 4: temperature/humidity sensor (SHT20) fault
 *           bit 3: VOC sensor (CCS811) fault, absent or not initialized
 *   [1]     battery: 0 = external power, else bits 7-4 capacity level 0-15,
 *           bits 3-0 voltage 1-14 (2.6 V .. 3.9 V), 15 = fault
 *   [2]     temperature (SHT20), °C + 32
 *   [3]     relative humidity, %
 *   [4..5]  eCO2, ppm   (0 = no measurement ready, 0xffff = sensor absent/failed)
 *   [6..7]  TVOC, ppb   (same special values)
 */

var PORT_STATUS = 103;
var PORT_CONFIG = 204;
var PORT_FRAG = 201;
var PORT_FWMAN = 203;

// configuration tags (firmware: tabs-airq/main.c, common/tabs/tabs.h)
var TAG_I_MINSEND_SEC = 0;  // len 2, s
var TAG_I_MINSEND_HR = 1;   // len 1, h
var TAG_T_DTEMP = 2;        // len 1, °C
var TAG_T_DHUMI = 3;        // len 1, %RH
var TAG_T_DCO2 = 4;         // len 1, %
var TAG_PORT = 128;         // len 1
var TAG_DEFAULTS = 129;     // len 0

var TAG_LEN = {};
TAG_LEN[TAG_I_MINSEND_SEC] = 2;
TAG_LEN[TAG_I_MINSEND_HR] = 1;
TAG_LEN[TAG_T_DTEMP] = 1;
TAG_LEN[TAG_T_DHUMI] = 1;
TAG_LEN[TAG_T_DCO2] = 1;
TAG_LEN[TAG_PORT] = 1;
TAG_LEN[TAG_DEFAULTS] = 0;

var VOC_NOT_READY = 0;
var VOC_NONE = 0xffff;

function readLE(bytes, off, len) {
  var v = 0;
  for (var i = len - 1; i >= 0; i--) {
    v = v * 256 + bytes[off + i];
  }
  return v;
}

function writeLE(out, v, len) {
  for (var i = 0; i < len; i++) {
    out.push(v % 256);
    v = Math.floor(v / 256);
  }
}

function hex(bytes, from, to) {
  var s = "";
  for (var i = from; i < to; i++) {
    s += (bytes[i] < 16 ? "0" : "") + bytes[i].toString(16);
  }
  return s;
}

function hex32(v) {
  var s = v.toString(16);
  while (s.length < 8) s = "0" + s;
  return "0x" + s;
}


// ------------------------------------------------
// Uplinks

function decodeStatus(bytes) {
  if (bytes.length < 8) {
    return { errors: ["status payload too short: expected 8 bytes, got " + bytes.length] };
  }
  var warnings = [];
  var data = {
    temp_sensor_fault: (bytes[0] & 0x10) !== 0,
    voc_sensor_fault: (bytes[0] & 0x08) !== 0
  };

  // battery
  var v = bytes[1] & 0x0f;
  if (bytes[1] === 0) {
    data.external_power = true;
  } else if (v === 15) {
    data.external_power = false;
    warnings.push("battery measurement fault");
  } else {
    data.external_power = false;
    data.battery_voltage = (25 + v) / 10;
    data.battery_percent = Math.round(((bytes[1] >> 4) & 0x0f) / 15 * 100);
    if (data.battery_percent <= 10) warnings.push("low battery");
  }

  // temperature/humidity (values of last good reading if the sensor failed)
  if (data.temp_sensor_fault) {
    warnings.push("temperature/humidity sensor fault");
  } else {
    data.temperature = (bytes[2] & 0x7f) - 32;
    data.humidity = bytes[3];
  }

  // air quality
  var co2 = readLE(bytes, 4, 2);
  var voc = readLE(bytes, 6, 2);
  if (co2 === VOC_NONE) {
    warnings.push("VOC sensor absent or failed");
  } else if (co2 === VOC_NOT_READY) {
    warnings.push("VOC sensor warming up, no measurement yet");
  } else {
    data.eco2 = co2;
    data.tvoc = voc;
  }

  var r = { data: data };
  if (warnings.length) r.warnings = warnings;
  return r;
}

function decodeConfig(bytes) {
  var cfg = {};
  var i = 0;
  while (i < bytes.length) {
    var tag = bytes[i++];
    var len = TAG_LEN[tag];
    if (len === undefined || i + len > bytes.length) {
      return { data: { config: cfg }, errors: ["config report: bad tag " + tag + " at offset " + (i - 1)] };
    }
    var v = readLE(bytes, i, len);
    i += len;
    switch (tag) {
      case TAG_I_MINSEND_SEC: cfg.min_send_interval_s = v; break;
      case TAG_I_MINSEND_HR: cfg.min_send_interval_s = v * 3600; break;
      case TAG_T_DTEMP: cfg.temperature_threshold = v; break;
      case TAG_T_DHUMI: cfg.humidity_threshold = v; break;
      case TAG_T_DCO2: cfg.eco2_threshold_percent = v; break;
      case TAG_PORT: cfg.port = v; break;
    }
  }
  return { data: { config: cfg } };
}

function decodeFrag(bytes) {
  var answers = [];
  var i = 0;
  while (i < bytes.length) {
    var c = bytes[i];
    if (c === 0x00 && i + 3 <= bytes.length) {
      answers.push({ cmd: "PackageVersionAns", package: bytes[i + 1], version: bytes[i + 2] });
      i += 3;
    } else if (c === 0x01 && i + 5 <= bytes.length) {
      var ri = readLE(bytes, i + 1, 2);
      answers.push({ cmd: "FragSessionStatusAns", session: ri >> 14, received: ri & 0x3fff,
        missing: bytes[i + 3], status: bytes[i + 4] });
      i += 5;
    } else if (c === 0x02 && i + 2 <= bytes.length) {
      var st = bytes[i + 1];
      var err = [];
      if (st & 1) err.push("encoding");
      if (st & 2) err.push("memory");
      if (st & 4) err.push("index");
      if (st & 8) err.push("descriptor");
      answers.push({ cmd: "FragSessionSetupAns", session: st >> 6, ok: err.length === 0, errors: err });
      i += 2;
    } else if (c === 0x03 && i + 2 <= bytes.length) {
      answers.push({ cmd: "FragSessionDeleteAns", session: bytes[i + 1] & 3,
        ok: (bytes[i + 1] & 4) === 0 });
      i += 2;
    } else {
      return { data: { frag: answers }, errors: ["frag: unknown answer " + hex(bytes, i, bytes.length)] };
    }
  }
  return { data: { frag: answers } };
}

function decodeFwman(bytes) {
  var answers = [];
  var i = 0;
  var IMG = ["none", "invalid", "mismatch", "valid"];
  while (i < bytes.length) {
    var c = bytes[i];
    if (c === 0x00 && i + 3 <= bytes.length) {
      answers.push({ cmd: "PackageVersionAns", package: bytes[i + 1], version: bytes[i + 2] });
      i += 3;
    } else if (c === 0x01 && i + 9 <= bytes.length) {
      answers.push({ cmd: "DevVersionAns", fw_crc: hex32(readLE(bytes, i + 1, 4)),
        hw_id: hex32(readLE(bytes, i + 5, 4)) });
      i += 9;
    } else if (c === 0x02 && i + 5 <= bytes.length) {
      answers.push({ cmd: "DevRebootTimeAns", time: readLE(bytes, i + 1, 4) });
      i += 5;
    } else if (c === 0x03 && i + 4 <= bytes.length) {
      answers.push({ cmd: "DevRebootCountdownAns", countdown: readLE(bytes, i + 1, 3) });
      i += 4;
    } else if (c === 0x04 && i + 2 <= bytes.length) {
      var a = { cmd: "DevUpgradeImageAns", status: IMG[bytes[i + 1]] || bytes[i + 1] };
      i += 2;
      if (a.status === "valid" && i + 4 <= bytes.length) {
        a.new_fw_crc = hex32(readLE(bytes, i, 4));
        i += 4;
      }
      answers.push(a);
    } else if (c === 0x05 && i + 2 <= bytes.length) {
      answers.push({ cmd: "DevDeleteImageAns", status: bytes[i + 1] });
      i += 2;
    } else {
      return { data: { fwman: answers }, errors: ["fwman: unknown answer " + hex(bytes, i, bytes.length)] };
    }
  }
  return { data: { fwman: answers } };
}

function decodeUplink(input) {
  var bytes = input.bytes || [];
  switch (input.fPort) {
    case PORT_STATUS: return decodeStatus(bytes);
    case PORT_CONFIG: return decodeConfig(bytes);
    case PORT_FRAG: return decodeFrag(bytes);
    case PORT_FWMAN: return decodeFwman(bytes);
    case 0: return { data: {} };
    default:
      return { data: { raw: hex(bytes, 0, bytes.length) }, warnings: ["unknown port " + input.fPort] };
  }
}


// ------------------------------------------------
// Downlinks

// Configuration (port 204), all fields optional:
//   defaults: true                   reset to defaults first
//   min_send_interval_s              heartbeat interval (0 = off, 60..65535 s, or whole hours up to 255 h)
//   temperature_threshold            send on temperature change >= this many °C (0..255, 0 = off)
//   humidity_threshold               send on humidity change >= this many %RH (0..255, 0 = off)
//   eco2_threshold_percent           send on eCO2 change > this many % (0..255, 0 = off)
//   port                             status uplink port (1..223)
// The device rejects a configuration with every trigger off, and answers every
// configuration downlink with a configuration report.
function encodeConfig(d, errors) {
  var out = [];
  function put(tag, v, len, min, max, name) {
    if (typeof v !== "number" || v !== Math.floor(v) || v < min || v > max) {
      errors.push(name + " must be an integer " + min + ".." + max);
      return;
    }
    out.push(tag);
    writeLE(out, v, len);
  }
  if (d.defaults) {
    out.push(TAG_DEFAULTS);
  }
  if (d.min_send_interval_s !== undefined) {
    var v = d.min_send_interval_s;
    if (v === 0) {
      put(TAG_I_MINSEND_SEC, 0, 2, 0, 0, "min_send_interval_s");
    } else if (v <= 0xffff) {
      put(TAG_I_MINSEND_SEC, v, 2, 60, 0xffff, "min_send_interval_s");
    } else if (v % 3600 === 0) {
      put(TAG_I_MINSEND_HR, v / 3600, 1, 1, 255, "min_send_interval_s (hours)");
    } else {
      errors.push("min_send_interval_s above 65535 must be a whole number of hours");
    }
  }
  if (d.temperature_threshold !== undefined) {
    put(TAG_T_DTEMP, d.temperature_threshold, 1, 0, 255, "temperature_threshold");
  }
  if (d.humidity_threshold !== undefined) {
    put(TAG_T_DHUMI, d.humidity_threshold, 1, 0, 255, "humidity_threshold");
  }
  if (d.eco2_threshold_percent !== undefined) {
    put(TAG_T_DCO2, d.eco2_threshold_percent, 1, 0, 255, "eco2_threshold_percent");
  }
  if (d.port !== undefined) {
    put(TAG_PORT, d.port, 1, 1, 223, "port");
  }
  return out;
}

function encodeDownlink(input) {
  var errors = [];
  var d = input.data || {};
  var bytes = encodeConfig(d, errors);
  if (errors.length) {
    return { errors: errors };
  }
  // an empty request just makes the device report its configuration
  return { bytes: bytes, fPort: PORT_CONFIG };
}
