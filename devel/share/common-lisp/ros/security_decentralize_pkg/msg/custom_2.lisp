; Auto-generated. Do not edit!


(cl:in-package security_decentralize_pkg-msg)


;//! \htmlinclude custom_2.msg.html

(cl:defclass <custom_2> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (location
    :reader location
    :initarg :location
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass custom_2 (<custom_2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <custom_2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'custom_2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name security_decentralize_pkg-msg:<custom_2> is deprecated: use security_decentralize_pkg-msg:custom_2 instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <custom_2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:robot_name-val is deprecated.  Use security_decentralize_pkg-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <custom_2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:location-val is deprecated.  Use security_decentralize_pkg-msg:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <custom_2>) ostream)
  "Serializes a message object of type '<custom_2>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <custom_2>) istream)
  "Deserializes a message object of type '<custom_2>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<custom_2>)))
  "Returns string type for a message object of type '<custom_2>"
  "security_decentralize_pkg/custom_2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'custom_2)))
  "Returns string type for a message object of type 'custom_2"
  "security_decentralize_pkg/custom_2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<custom_2>)))
  "Returns md5sum for a message object of type '<custom_2>"
  "fc4b6bfc08bf9f97dcec3e07cd0e259b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'custom_2)))
  "Returns md5sum for a message object of type 'custom_2"
  "fc4b6bfc08bf9f97dcec3e07cd0e259b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<custom_2>)))
  "Returns full string definition for message of type '<custom_2>"
  (cl:format cl:nil "std_msgs/String robot_name~%geometry_msgs/Pose2D location~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'custom_2)))
  "Returns full string definition for message of type 'custom_2"
  (cl:format cl:nil "std_msgs/String robot_name~%geometry_msgs/Pose2D location~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <custom_2>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <custom_2>))
  "Converts a ROS message object to a list"
  (cl:list 'custom_2
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':location (location msg))
))
