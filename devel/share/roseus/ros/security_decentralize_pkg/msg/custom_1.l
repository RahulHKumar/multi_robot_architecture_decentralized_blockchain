;; Auto-generated. Do not edit!


(when (boundp 'security_decentralize_pkg::custom_1)
  (if (not (find-package "SECURITY_DECENTRALIZE_PKG"))
    (make-package "SECURITY_DECENTRALIZE_PKG"))
  (shadow 'custom_1 (find-package "SECURITY_DECENTRALIZE_PKG")))
(unless (find-package "SECURITY_DECENTRALIZE_PKG::CUSTOM_1")
  (make-package "SECURITY_DECENTRALIZE_PKG::CUSTOM_1"))

(in-package "ROS")
;;//! \htmlinclude custom_1.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass security_decentralize_pkg::custom_1
  :super ros::object
  :slots (_robot_name _location ))

(defmethod security_decentralize_pkg::custom_1
  (:init
   (&key
    ((:robot_name __robot_name) (instance std_msgs::String :init))
    ((:location __location) (instance geometry_msgs::Pose2D :init))
    )
   (send-super :init)
   (setq _robot_name __robot_name)
   (setq _location __location)
   self)
  (:robot_name
   (&rest __robot_name)
   (if (keywordp (car __robot_name))
       (send* _robot_name __robot_name)
     (progn
       (if __robot_name (setq _robot_name (car __robot_name)))
       _robot_name)))
  (:location
   (&rest __location)
   (if (keywordp (car __location))
       (send* _location __location)
     (progn
       (if __location (setq _location (car __location)))
       _location)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _robot_name
    (send _robot_name :serialization-length)
    ;; geometry_msgs/Pose2D _location
    (send _location :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _robot_name
       (send _robot_name :serialize s)
     ;; geometry_msgs/Pose2D _location
       (send _location :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _robot_name
     (send _robot_name :deserialize buf ptr-) (incf ptr- (send _robot_name :serialization-length))
   ;; geometry_msgs/Pose2D _location
     (send _location :deserialize buf ptr-) (incf ptr- (send _location :serialization-length))
   ;;
   self)
  )

(setf (get security_decentralize_pkg::custom_1 :md5sum-) "fc4b6bfc08bf9f97dcec3e07cd0e259b")
(setf (get security_decentralize_pkg::custom_1 :datatype-) "security_decentralize_pkg/custom_1")
(setf (get security_decentralize_pkg::custom_1 :definition-)
      "std_msgs/String robot_name
geometry_msgs/Pose2D location

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

")



(provide :security_decentralize_pkg/custom_1 "fc4b6bfc08bf9f97dcec3e07cd0e259b")


