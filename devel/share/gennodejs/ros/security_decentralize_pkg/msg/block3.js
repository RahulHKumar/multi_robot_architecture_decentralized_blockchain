// Auto-generated. Do not edit!

// (in-package security_decentralize_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class block3 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_hash = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_hash')) {
        this.robot_hash = initObj.robot_hash
      }
      else {
        this.robot_hash = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type block3
    // Serialize message field [robot_hash]
    bufferOffset = std_msgs.msg.String.serialize(obj.robot_hash, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type block3
    let len;
    let data = new block3(null);
    // Deserialize message field [robot_hash]
    data.robot_hash = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.robot_hash);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'security_decentralize_pkg/block3';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df6427b3d0603bcc36155fb56a25758c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String robot_hash
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new block3(null);
    if (msg.robot_hash !== undefined) {
      resolved.robot_hash = std_msgs.msg.String.Resolve(msg.robot_hash)
    }
    else {
      resolved.robot_hash = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = block3;
