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

class pp1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_name = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pp1
    // Serialize message field [robot_name]
    bufferOffset = std_msgs.msg.String.serialize(obj.robot_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pp1
    let len;
    let data = new pp1(null);
    // Deserialize message field [robot_name]
    data.robot_name = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.robot_name);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'security_decentralize_pkg/pp1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a714a45eb282cdb6b20eb8ead90c5b9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String robot_name
    
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
    const resolved = new pp1(null);
    if (msg.robot_name !== undefined) {
      resolved.robot_name = std_msgs.msg.String.Resolve(msg.robot_name)
    }
    else {
      resolved.robot_name = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = pp1;
