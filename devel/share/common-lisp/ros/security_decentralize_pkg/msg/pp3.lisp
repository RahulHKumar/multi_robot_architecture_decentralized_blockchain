; Auto-generated. Do not edit!


(cl:in-package security_decentralize_pkg-msg)


;//! \htmlinclude pp3.msg.html

(cl:defclass <pp3> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass pp3 (<pp3>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pp3>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pp3)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name security_decentralize_pkg-msg:<pp3> is deprecated: use security_decentralize_pkg-msg:pp3 instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <pp3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:robot_name-val is deprecated.  Use security_decentralize_pkg-msg:robot_name instead.")
  (robot_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pp3>) ostream)
  "Serializes a message object of type '<pp3>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_name) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pp3>) istream)
  "Deserializes a message object of type '<pp3>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_name) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pp3>)))
  "Returns string type for a message object of type '<pp3>"
  "security_decentralize_pkg/pp3")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pp3)))
  "Returns string type for a message object of type 'pp3"
  "security_decentralize_pkg/pp3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pp3>)))
  "Returns md5sum for a message object of type '<pp3>"
  "a714a45eb282cdb6b20eb8ead90c5b9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pp3)))
  "Returns md5sum for a message object of type 'pp3"
  "a714a45eb282cdb6b20eb8ead90c5b9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pp3>)))
  "Returns full string definition for message of type '<pp3>"
  (cl:format cl:nil "std_msgs/String robot_name~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pp3)))
  "Returns full string definition for message of type 'pp3"
  (cl:format cl:nil "std_msgs/String robot_name~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pp3>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pp3>))
  "Converts a ROS message object to a list"
  (cl:list 'pp3
    (cl:cons ':robot_name (robot_name msg))
))
