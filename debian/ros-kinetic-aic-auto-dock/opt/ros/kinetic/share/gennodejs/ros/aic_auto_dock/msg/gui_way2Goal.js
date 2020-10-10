// Auto-generated. Do not edit!

// (in-package aic_auto_dock.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class gui_way2Goal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tag_no = null;
      this.type = null;
      this.pose = null;
      this.vel_line = null;
      this.vel_angle = null;
      this.back_dist = null;
      this.obstacle_dist = null;
      this.preparePosition = null;
      this.scale = null;
    }
    else {
      if (initObj.hasOwnProperty('tag_no')) {
        this.tag_no = initObj.tag_no
      }
      else {
        this.tag_no = '';
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('vel_line')) {
        this.vel_line = initObj.vel_line
      }
      else {
        this.vel_line = 0.0;
      }
      if (initObj.hasOwnProperty('vel_angle')) {
        this.vel_angle = initObj.vel_angle
      }
      else {
        this.vel_angle = 0.0;
      }
      if (initObj.hasOwnProperty('back_dist')) {
        this.back_dist = initObj.back_dist
      }
      else {
        this.back_dist = 0.0;
      }
      if (initObj.hasOwnProperty('obstacle_dist')) {
        this.obstacle_dist = initObj.obstacle_dist
      }
      else {
        this.obstacle_dist = 0.0;
      }
      if (initObj.hasOwnProperty('preparePosition')) {
        this.preparePosition = initObj.preparePosition
      }
      else {
        this.preparePosition = 0.0;
      }
      if (initObj.hasOwnProperty('scale')) {
        this.scale = initObj.scale
      }
      else {
        this.scale = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gui_way2Goal
    // Serialize message field [tag_no]
    bufferOffset = _serializer.string(obj.tag_no, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [vel_line]
    bufferOffset = _serializer.float32(obj.vel_line, buffer, bufferOffset);
    // Serialize message field [vel_angle]
    bufferOffset = _serializer.float32(obj.vel_angle, buffer, bufferOffset);
    // Serialize message field [back_dist]
    bufferOffset = _serializer.float32(obj.back_dist, buffer, bufferOffset);
    // Serialize message field [obstacle_dist]
    bufferOffset = _serializer.float32(obj.obstacle_dist, buffer, bufferOffset);
    // Serialize message field [preparePosition]
    bufferOffset = _serializer.float32(obj.preparePosition, buffer, bufferOffset);
    // Serialize message field [scale]
    bufferOffset = _serializer.float32(obj.scale, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gui_way2Goal
    let len;
    let data = new gui_way2Goal(null);
    // Deserialize message field [tag_no]
    data.tag_no = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel_line]
    data.vel_line = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vel_angle]
    data.vel_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [back_dist]
    data.back_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [obstacle_dist]
    data.obstacle_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [preparePosition]
    data.preparePosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [scale]
    data.scale = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.tag_no.length;
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aic_auto_dock/gui_way2Goal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0df86edf95e2372a391a8bf1928e52fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    string              tag_no
    int32               type
    geometry_msgs/Pose  pose
    float32             vel_line
    float32             vel_angle
    float32             back_dist
    float32             obstacle_dist
    float32             preparePosition
    float32             scale            #角速度响应比例，取值范围：0~0.1
    
    int32               BACK = 0
    int32               STRAIGHT = 1
    int32               PAUSE = 2
    int32               RESUM = 3
    int32               INITPORT = 4
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gui_way2Goal(null);
    if (msg.tag_no !== undefined) {
      resolved.tag_no = msg.tag_no;
    }
    else {
      resolved.tag_no = ''
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.vel_line !== undefined) {
      resolved.vel_line = msg.vel_line;
    }
    else {
      resolved.vel_line = 0.0
    }

    if (msg.vel_angle !== undefined) {
      resolved.vel_angle = msg.vel_angle;
    }
    else {
      resolved.vel_angle = 0.0
    }

    if (msg.back_dist !== undefined) {
      resolved.back_dist = msg.back_dist;
    }
    else {
      resolved.back_dist = 0.0
    }

    if (msg.obstacle_dist !== undefined) {
      resolved.obstacle_dist = msg.obstacle_dist;
    }
    else {
      resolved.obstacle_dist = 0.0
    }

    if (msg.preparePosition !== undefined) {
      resolved.preparePosition = msg.preparePosition;
    }
    else {
      resolved.preparePosition = 0.0
    }

    if (msg.scale !== undefined) {
      resolved.scale = msg.scale;
    }
    else {
      resolved.scale = 0.0
    }

    return resolved;
    }
};

// Constants for message
gui_way2Goal.Constants = {
  BACK: 0,
  STRAIGHT: 1,
  PAUSE: 2,
  RESUM: 3,
  INITPORT: 4,
}

module.exports = gui_way2Goal;
