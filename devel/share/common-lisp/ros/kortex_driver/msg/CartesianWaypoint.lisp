; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianWaypoint.msg.html

(cl:defclass <CartesianWaypoint> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type kortex_driver-msg:Pose
    :initform (cl:make-instance 'kortex_driver-msg:Pose))
   (reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:integer
    :initform 0)
   (maximum_linear_velocity
    :reader maximum_linear_velocity
    :initarg :maximum_linear_velocity
    :type cl:float
    :initform 0.0)
   (maximum_angular_velocity
    :reader maximum_angular_velocity
    :initarg :maximum_angular_velocity
    :type cl:float
    :initform 0.0)
   (blending_radius
    :reader blending_radius
    :initarg :blending_radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass CartesianWaypoint (<CartesianWaypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianWaypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianWaypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianWaypoint> is deprecated: use kortex_driver-msg:CartesianWaypoint instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <CartesianWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:pose-val is deprecated.  Use kortex_driver-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <CartesianWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:reference_frame-val is deprecated.  Use kortex_driver-msg:reference_frame instead.")
  (reference_frame m))

(cl:ensure-generic-function 'maximum_linear_velocity-val :lambda-list '(m))
(cl:defmethod maximum_linear_velocity-val ((m <CartesianWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:maximum_linear_velocity-val is deprecated.  Use kortex_driver-msg:maximum_linear_velocity instead.")
  (maximum_linear_velocity m))

(cl:ensure-generic-function 'maximum_angular_velocity-val :lambda-list '(m))
(cl:defmethod maximum_angular_velocity-val ((m <CartesianWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:maximum_angular_velocity-val is deprecated.  Use kortex_driver-msg:maximum_angular_velocity instead.")
  (maximum_angular_velocity m))

(cl:ensure-generic-function 'blending_radius-val :lambda-list '(m))
(cl:defmethod blending_radius-val ((m <CartesianWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:blending_radius-val is deprecated.  Use kortex_driver-msg:blending_radius instead.")
  (blending_radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianWaypoint>) ostream)
  "Serializes a message object of type '<CartesianWaypoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maximum_linear_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maximum_angular_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'blending_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianWaypoint>) istream)
  "Deserializes a message object of type '<CartesianWaypoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maximum_linear_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maximum_angular_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'blending_radius) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianWaypoint>)))
  "Returns string type for a message object of type '<CartesianWaypoint>"
  "kortex_driver/CartesianWaypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianWaypoint)))
  "Returns string type for a message object of type 'CartesianWaypoint"
  "kortex_driver/CartesianWaypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianWaypoint>)))
  "Returns md5sum for a message object of type '<CartesianWaypoint>"
  "057d3bd32497f1e96612efc0fb6ef690")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianWaypoint)))
  "Returns md5sum for a message object of type 'CartesianWaypoint"
  "057d3bd32497f1e96612efc0fb6ef690")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianWaypoint>)))
  "Returns full string definition for message of type '<CartesianWaypoint>"
  (cl:format cl:nil "~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianWaypoint)))
  "Returns full string definition for message of type 'CartesianWaypoint"
  (cl:format cl:nil "~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianWaypoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianWaypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianWaypoint
    (cl:cons ':pose (pose msg))
    (cl:cons ':reference_frame (reference_frame msg))
    (cl:cons ':maximum_linear_velocity (maximum_linear_velocity msg))
    (cl:cons ':maximum_angular_velocity (maximum_angular_velocity msg))
    (cl:cons ':blending_radius (blending_radius msg))
))
