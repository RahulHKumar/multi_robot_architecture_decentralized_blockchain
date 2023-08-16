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

class malicious_flags {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bot1_mal = null;
      this.bot2_mal = null;
      this.bot3_mal = null;
    }
    else {
      if (initObj.hasOwnProperty('bot1_mal')) {
        this.bot1_mal = initObj.bot1_mal
      }
      else {
        this.bot1_mal = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('bot2_mal')) {
        this.bot2_mal = initObj.bot2_mal
      }
      else {
        this.bot2_mal = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('bot3_mal')) {
        this.bot3_mal = initObj.bot3_mal
      }
      else {
        this.bot3_mal = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type malicious_flags
    // Serialize message field [bot1_mal]
    bufferOffset = std_msgs.msg.String.serialize(obj.bot1_mal, buffer, bufferOffset);
    // Serialize message field [bot2_mal]
    bufferOffset = std_msgs.msg.String.serialize(obj.bot2_mal, buffer, bufferOffset);
    // Serialize message field [bot3_mal]
    bufferOffset = std_msgs.msg.String.serialize(obj.bot3_mal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type malicious_flags
    let len;
    let data = new malicious_flags(null);
    // Deserialize message field [bot1_mal]
    data.bot1_mal = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [bot2_mal]
    data.bot2_mal = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [bot3_mal]
    data.bot3_mal = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.bot1_mal);
    length += std_msgs.msg.String.getMessageSize(object.bot2_mal);
    length += std_msgs.msg.String.getMessageSize(object.bot3_mal);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'security_decentralize_pkg/malicious_flags';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1fbd4377f4fa1942d302cc06fa5d8c8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String bot1_mal
    std_msgs/String bot2_mal
    std_msgs/String bot3_mal
    
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
    const resolved = new malicious_flags(null);
    if (msg.bot1_mal !== undefined) {
      resolved.bot1_mal = std_msgs.msg.String.Resolve(msg.bot1_mal)
    }
    else {
      resolved.bot1_mal = new std_msgs.msg.String()
    }

    if (msg.bot2_mal !== undefined) {
      resolved.bot2_mal = std_msgs.msg.String.Resolve(msg.bot2_mal)
    }
    else {
      resolved.bot2_mal = new std_msgs.msg.String()
    }

    if (msg.bot3_mal !== undefined) {
      resolved.bot3_mal = std_msgs.msg.String.Resolve(msg.bot3_mal)
    }
    else {
      resolved.bot3_mal = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = malicious_flags;
