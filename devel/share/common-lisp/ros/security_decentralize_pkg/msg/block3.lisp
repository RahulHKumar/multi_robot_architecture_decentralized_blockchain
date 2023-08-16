; Auto-generated. Do not edit!


(cl:in-package security_decentralize_pkg-msg)


;//! \htmlinclude block3.msg.html

(cl:defclass <block3> (roslisp-msg-protocol:ros-message)
  ((robot_hash
    :reader robot_hash
    :initarg :robot_hash
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass block3 (<block3>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <block3>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'block3)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name security_decentralize_pkg-msg:<block3> is deprecated: use security_decentralize_pkg-msg:block3 instead.")))

(cl:ensure-generic-function 'robot_hash-val :lambda-list '(m))
(cl:defmethod robot_hash-val ((m <block3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:robot_hash-val is deprecated.  Use security_decentralize_pkg-msg:robot_hash instead.")
  (robot_hash m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <block3>) ostream)
  "Serializes a message object of type '<block3>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_hash) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <block3>) istream)
  "Deserializes a message object of type '<block3>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_hash) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<block3>)))
  "Returns string type for a message object of type '<block3>"
  "security_decentralize_pkg/block3")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'block3)))
  "Returns string type for a message object of type 'block3"
  "security_decentralize_pkg/block3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<block3>)))
  "Returns md5sum for a message object of type '<block3>"
  "df6427b3d0603bcc36155fb56a25758c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'block3)))
  "Returns md5sum for a message object of type 'block3"
  "df6427b3d0603bcc36155fb56a25758c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<block3>)))
  "Returns full string definition for message of type '<block3>"
  (cl:format cl:nil "std_msgs/String robot_hash~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'block3)))
  "Returns full string definition for message of type 'block3"
  (cl:format cl:nil "std_msgs/String robot_hash~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <block3>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_hash))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <block3>))
  "Converts a ROS message object to a list"
  (cl:list 'block3
    (cl:cons ':robot_hash (robot_hash msg))
))
