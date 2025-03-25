// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SequenceHandle = require('./SequenceHandle.js');
let ActionHandle = require('./ActionHandle.js');
let MappingHandle = require('./MappingHandle.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let MapHandle = require('./MapHandle.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let SafetyHandle = require('./SafetyHandle.js');
let NetworkHandle = require('./NetworkHandle.js');
let Ssid = require('./Ssid.js');
let ControllerHandle = require('./ControllerHandle.js');

//-----------------------------------------------------------

class ConfigurationChangeNotification_configuration_change {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sequence_handle = null;
      this.action_handle = null;
      this.mapping_handle = null;
      this.map_group_handle = null;
      this.map_handle = null;
      this.user_profile_handle = null;
      this.protection_zone_handle = null;
      this.safety_handle = null;
      this.network_handle = null;
      this.ssid = null;
      this.controller_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('sequence_handle')) {
        this.sequence_handle = initObj.sequence_handle
      }
      else {
        this.sequence_handle = [];
      }
      if (initObj.hasOwnProperty('action_handle')) {
        this.action_handle = initObj.action_handle
      }
      else {
        this.action_handle = [];
      }
      if (initObj.hasOwnProperty('mapping_handle')) {
        this.mapping_handle = initObj.mapping_handle
      }
      else {
        this.mapping_handle = [];
      }
      if (initObj.hasOwnProperty('map_group_handle')) {
        this.map_group_handle = initObj.map_group_handle
      }
      else {
        this.map_group_handle = [];
      }
      if (initObj.hasOwnProperty('map_handle')) {
        this.map_handle = initObj.map_handle
      }
      else {
        this.map_handle = [];
      }
      if (initObj.hasOwnProperty('user_profile_handle')) {
        this.user_profile_handle = initObj.user_profile_handle
      }
      else {
        this.user_profile_handle = [];
      }
      if (initObj.hasOwnProperty('protection_zone_handle')) {
        this.protection_zone_handle = initObj.protection_zone_handle
      }
      else {
        this.protection_zone_handle = [];
      }
      if (initObj.hasOwnProperty('safety_handle')) {
        this.safety_handle = initObj.safety_handle
      }
      else {
        this.safety_handle = [];
      }
      if (initObj.hasOwnProperty('network_handle')) {
        this.network_handle = initObj.network_handle
      }
      else {
        this.network_handle = [];
      }
      if (initObj.hasOwnProperty('ssid')) {
        this.ssid = initObj.ssid
      }
      else {
        this.ssid = [];
      }
      if (initObj.hasOwnProperty('controller_handle')) {
        this.controller_handle = initObj.controller_handle
      }
      else {
        this.controller_handle = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigurationChangeNotification_configuration_change
    // Serialize message field [sequence_handle]
    // Serialize the length for message field [sequence_handle]
    bufferOffset = _serializer.uint32(obj.sequence_handle.length, buffer, bufferOffset);
    obj.sequence_handle.forEach((val) => {
      bufferOffset = SequenceHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [action_handle]
    // Serialize the length for message field [action_handle]
    bufferOffset = _serializer.uint32(obj.action_handle.length, buffer, bufferOffset);
    obj.action_handle.forEach((val) => {
      bufferOffset = ActionHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [mapping_handle]
    // Serialize the length for message field [mapping_handle]
    bufferOffset = _serializer.uint32(obj.mapping_handle.length, buffer, bufferOffset);
    obj.mapping_handle.forEach((val) => {
      bufferOffset = MappingHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [map_group_handle]
    // Serialize the length for message field [map_group_handle]
    bufferOffset = _serializer.uint32(obj.map_group_handle.length, buffer, bufferOffset);
    obj.map_group_handle.forEach((val) => {
      bufferOffset = MapGroupHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [map_handle]
    // Serialize the length for message field [map_handle]
    bufferOffset = _serializer.uint32(obj.map_handle.length, buffer, bufferOffset);
    obj.map_handle.forEach((val) => {
      bufferOffset = MapHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [user_profile_handle]
    // Serialize the length for message field [user_profile_handle]
    bufferOffset = _serializer.uint32(obj.user_profile_handle.length, buffer, bufferOffset);
    obj.user_profile_handle.forEach((val) => {
      bufferOffset = UserProfileHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [protection_zone_handle]
    // Serialize the length for message field [protection_zone_handle]
    bufferOffset = _serializer.uint32(obj.protection_zone_handle.length, buffer, bufferOffset);
    obj.protection_zone_handle.forEach((val) => {
      bufferOffset = ProtectionZoneHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [safety_handle]
    // Serialize the length for message field [safety_handle]
    bufferOffset = _serializer.uint32(obj.safety_handle.length, buffer, bufferOffset);
    obj.safety_handle.forEach((val) => {
      bufferOffset = SafetyHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [network_handle]
    // Serialize the length for message field [network_handle]
    bufferOffset = _serializer.uint32(obj.network_handle.length, buffer, bufferOffset);
    obj.network_handle.forEach((val) => {
      bufferOffset = NetworkHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [ssid]
    // Serialize the length for message field [ssid]
    bufferOffset = _serializer.uint32(obj.ssid.length, buffer, bufferOffset);
    obj.ssid.forEach((val) => {
      bufferOffset = Ssid.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [controller_handle]
    // Serialize the length for message field [controller_handle]
    bufferOffset = _serializer.uint32(obj.controller_handle.length, buffer, bufferOffset);
    obj.controller_handle.forEach((val) => {
      bufferOffset = ControllerHandle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigurationChangeNotification_configuration_change
    let len;
    let data = new ConfigurationChangeNotification_configuration_change(null);
    // Deserialize message field [sequence_handle]
    // Deserialize array length for message field [sequence_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sequence_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sequence_handle[i] = SequenceHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [action_handle]
    // Deserialize array length for message field [action_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.action_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.action_handle[i] = ActionHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [mapping_handle]
    // Deserialize array length for message field [mapping_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mapping_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mapping_handle[i] = MappingHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [map_group_handle]
    // Deserialize array length for message field [map_group_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.map_group_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.map_group_handle[i] = MapGroupHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [map_handle]
    // Deserialize array length for message field [map_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.map_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.map_handle[i] = MapHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [user_profile_handle]
    // Deserialize array length for message field [user_profile_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.user_profile_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.user_profile_handle[i] = UserProfileHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [protection_zone_handle]
    // Deserialize array length for message field [protection_zone_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.protection_zone_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.protection_zone_handle[i] = ProtectionZoneHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [safety_handle]
    // Deserialize array length for message field [safety_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.safety_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.safety_handle[i] = SafetyHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [network_handle]
    // Deserialize array length for message field [network_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.network_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.network_handle[i] = NetworkHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [ssid]
    // Deserialize array length for message field [ssid]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ssid = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ssid[i] = Ssid.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [controller_handle]
    // Deserialize array length for message field [controller_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controller_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controller_handle[i] = ControllerHandle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.sequence_handle.length;
    length += 12 * object.action_handle.length;
    length += 8 * object.mapping_handle.length;
    length += 8 * object.map_group_handle.length;
    length += 8 * object.map_handle.length;
    length += 8 * object.user_profile_handle.length;
    length += 8 * object.protection_zone_handle.length;
    length += 4 * object.safety_handle.length;
    length += 4 * object.network_handle.length;
    object.ssid.forEach((val) => {
      length += Ssid.getMessageSize(val);
    });
    length += 8 * object.controller_handle.length;
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ConfigurationChangeNotification_configuration_change';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da6a00bf2364ba1e44afbbbd7c77db5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SequenceHandle[] sequence_handle
    ActionHandle[] action_handle
    MappingHandle[] mapping_handle
    MapGroupHandle[] map_group_handle
    MapHandle[] map_handle
    UserProfileHandle[] user_profile_handle
    ProtectionZoneHandle[] protection_zone_handle
    SafetyHandle[] safety_handle
    NetworkHandle[] network_handle
    Ssid[] ssid
    ControllerHandle[] controller_handle
    ================================================================================
    MSG: kortex_driver/SequenceHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/ActionHandle
    
    uint32 identifier
    uint32 action_type
    uint32 permission
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/MapGroupHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/MapHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/ProtectionZoneHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/NetworkHandle
    
    uint32 type
    ================================================================================
    MSG: kortex_driver/Ssid
    
    string identifier
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigurationChangeNotification_configuration_change(null);
    if (msg.sequence_handle !== undefined) {
      resolved.sequence_handle = new Array(msg.sequence_handle.length);
      for (let i = 0; i < resolved.sequence_handle.length; ++i) {
        resolved.sequence_handle[i] = SequenceHandle.Resolve(msg.sequence_handle[i]);
      }
    }
    else {
      resolved.sequence_handle = []
    }

    if (msg.action_handle !== undefined) {
      resolved.action_handle = new Array(msg.action_handle.length);
      for (let i = 0; i < resolved.action_handle.length; ++i) {
        resolved.action_handle[i] = ActionHandle.Resolve(msg.action_handle[i]);
      }
    }
    else {
      resolved.action_handle = []
    }

    if (msg.mapping_handle !== undefined) {
      resolved.mapping_handle = new Array(msg.mapping_handle.length);
      for (let i = 0; i < resolved.mapping_handle.length; ++i) {
        resolved.mapping_handle[i] = MappingHandle.Resolve(msg.mapping_handle[i]);
      }
    }
    else {
      resolved.mapping_handle = []
    }

    if (msg.map_group_handle !== undefined) {
      resolved.map_group_handle = new Array(msg.map_group_handle.length);
      for (let i = 0; i < resolved.map_group_handle.length; ++i) {
        resolved.map_group_handle[i] = MapGroupHandle.Resolve(msg.map_group_handle[i]);
      }
    }
    else {
      resolved.map_group_handle = []
    }

    if (msg.map_handle !== undefined) {
      resolved.map_handle = new Array(msg.map_handle.length);
      for (let i = 0; i < resolved.map_handle.length; ++i) {
        resolved.map_handle[i] = MapHandle.Resolve(msg.map_handle[i]);
      }
    }
    else {
      resolved.map_handle = []
    }

    if (msg.user_profile_handle !== undefined) {
      resolved.user_profile_handle = new Array(msg.user_profile_handle.length);
      for (let i = 0; i < resolved.user_profile_handle.length; ++i) {
        resolved.user_profile_handle[i] = UserProfileHandle.Resolve(msg.user_profile_handle[i]);
      }
    }
    else {
      resolved.user_profile_handle = []
    }

    if (msg.protection_zone_handle !== undefined) {
      resolved.protection_zone_handle = new Array(msg.protection_zone_handle.length);
      for (let i = 0; i < resolved.protection_zone_handle.length; ++i) {
        resolved.protection_zone_handle[i] = ProtectionZoneHandle.Resolve(msg.protection_zone_handle[i]);
      }
    }
    else {
      resolved.protection_zone_handle = []
    }

    if (msg.safety_handle !== undefined) {
      resolved.safety_handle = new Array(msg.safety_handle.length);
      for (let i = 0; i < resolved.safety_handle.length; ++i) {
        resolved.safety_handle[i] = SafetyHandle.Resolve(msg.safety_handle[i]);
      }
    }
    else {
      resolved.safety_handle = []
    }

    if (msg.network_handle !== undefined) {
      resolved.network_handle = new Array(msg.network_handle.length);
      for (let i = 0; i < resolved.network_handle.length; ++i) {
        resolved.network_handle[i] = NetworkHandle.Resolve(msg.network_handle[i]);
      }
    }
    else {
      resolved.network_handle = []
    }

    if (msg.ssid !== undefined) {
      resolved.ssid = new Array(msg.ssid.length);
      for (let i = 0; i < resolved.ssid.length; ++i) {
        resolved.ssid[i] = Ssid.Resolve(msg.ssid[i]);
      }
    }
    else {
      resolved.ssid = []
    }

    if (msg.controller_handle !== undefined) {
      resolved.controller_handle = new Array(msg.controller_handle.length);
      for (let i = 0; i < resolved.controller_handle.length; ++i) {
        resolved.controller_handle[i] = ControllerHandle.Resolve(msg.controller_handle[i]);
      }
    }
    else {
      resolved.controller_handle = []
    }

    return resolved;
    }
};

module.exports = ConfigurationChangeNotification_configuration_change;
