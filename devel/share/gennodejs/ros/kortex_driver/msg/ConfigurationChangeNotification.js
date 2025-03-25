// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Timestamp = require('./Timestamp.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let Connection = require('./Connection.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');

//-----------------------------------------------------------

class ConfigurationChangeNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event = null;
      this.timestamp = null;
      this.user_handle = null;
      this.connection = null;
      this.oneof_configuration_change = null;
    }
    else {
      if (initObj.hasOwnProperty('event')) {
        this.event = initObj.event
      }
      else {
        this.event = 0;
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = new Timestamp();
      }
      if (initObj.hasOwnProperty('user_handle')) {
        this.user_handle = initObj.user_handle
      }
      else {
        this.user_handle = new UserProfileHandle();
      }
      if (initObj.hasOwnProperty('connection')) {
        this.connection = initObj.connection
      }
      else {
        this.connection = new Connection();
      }
      if (initObj.hasOwnProperty('oneof_configuration_change')) {
        this.oneof_configuration_change = initObj.oneof_configuration_change
      }
      else {
        this.oneof_configuration_change = new ConfigurationChangeNotification_configuration_change();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigurationChangeNotification
    // Serialize message field [event]
    bufferOffset = _serializer.uint32(obj.event, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = Timestamp.serialize(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    // Serialize message field [oneof_configuration_change]
    bufferOffset = ConfigurationChangeNotification_configuration_change.serialize(obj.oneof_configuration_change, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigurationChangeNotification
    let len;
    let data = new ConfigurationChangeNotification(null);
    // Deserialize message field [event]
    data.event = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    // Deserialize message field [oneof_configuration_change]
    data.oneof_configuration_change = ConfigurationChangeNotification_configuration_change.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    length += ConfigurationChangeNotification_configuration_change.getMessageSize(object.oneof_configuration_change);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ConfigurationChangeNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3adec35b48ab707b96b990bb86718c1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 event
    Timestamp timestamp
    UserProfileHandle user_handle
    Connection connection
    ConfigurationChangeNotification_configuration_change oneof_configuration_change
    ================================================================================
    MSG: kortex_driver/Timestamp
    
    uint32 sec
    uint32 usec
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/Connection
    
    UserProfileHandle user_handle
    string connection_information
    uint32 connection_identifier
    ================================================================================
    MSG: kortex_driver/ConfigurationChangeNotification_configuration_change
    
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
    const resolved = new ConfigurationChangeNotification(null);
    if (msg.event !== undefined) {
      resolved.event = msg.event;
    }
    else {
      resolved.event = 0
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = Timestamp.Resolve(msg.timestamp)
    }
    else {
      resolved.timestamp = new Timestamp()
    }

    if (msg.user_handle !== undefined) {
      resolved.user_handle = UserProfileHandle.Resolve(msg.user_handle)
    }
    else {
      resolved.user_handle = new UserProfileHandle()
    }

    if (msg.connection !== undefined) {
      resolved.connection = Connection.Resolve(msg.connection)
    }
    else {
      resolved.connection = new Connection()
    }

    if (msg.oneof_configuration_change !== undefined) {
      resolved.oneof_configuration_change = ConfigurationChangeNotification_configuration_change.Resolve(msg.oneof_configuration_change)
    }
    else {
      resolved.oneof_configuration_change = new ConfigurationChangeNotification_configuration_change()
    }

    return resolved;
    }
};

module.exports = ConfigurationChangeNotification;
