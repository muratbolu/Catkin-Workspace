// Auto-generated. Do not edit!

// (in-package ros_exercises.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class check_palindromeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type check_palindromeRequest
    // Serialize message field [input]
    bufferOffset = _serializer.string(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type check_palindromeRequest
    let len;
    let data = new check_palindromeRequest(null);
    // Deserialize message field [input]
    data.input = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.input);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_exercises/check_palindromeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39e92f1778057359c64c7b8a7d7b19de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string input
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new check_palindromeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = ''
    }

    return resolved;
    }
};

class check_palindromeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bit = null;
    }
    else {
      if (initObj.hasOwnProperty('bit')) {
        this.bit = initObj.bit
      }
      else {
        this.bit = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type check_palindromeResponse
    // Serialize message field [bit]
    bufferOffset = _serializer.int8(obj.bit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type check_palindromeResponse
    let len;
    let data = new check_palindromeResponse(null);
    // Deserialize message field [bit]
    data.bit = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_exercises/check_palindromeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ef3392b71b4bc9968c0cbe6d4880901';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 bit
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new check_palindromeResponse(null);
    if (msg.bit !== undefined) {
      resolved.bit = msg.bit;
    }
    else {
      resolved.bit = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: check_palindromeRequest,
  Response: check_palindromeResponse,
  md5sum() { return '9435240a40d8ae413a3b7925d764bbcf'; },
  datatype() { return 'ros_exercises/check_palindrome'; }
};
