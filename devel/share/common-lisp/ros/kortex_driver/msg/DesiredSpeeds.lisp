; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DesiredSpeeds.msg.html

(cl:defclass <DesiredSpeeds> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0)
   (joint_speed
    :reader joint_speed
    :initarg :joint_speed
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DesiredSpeeds (<DesiredSpeeds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DesiredSpeeds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DesiredSpeeds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DesiredSpeeds> is deprecated: use kortex_driver-msg:DesiredSpeeds instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <DesiredSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:linear-val is deprecated.  Use kortex_driver-msg:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <DesiredSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:angular-val is deprecated.  Use kortex_driver-msg:angular instead.")
  (angular m))

(cl:ensure-generic-function 'joint_speed-val :lambda-list '(m))
(cl:defmethod joint_speed-val ((m <DesiredSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_speed-val is deprecated.  Use kortex_driver-msg:joint_speed instead.")
  (joint_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DesiredSpeeds>) ostream)
  "Serializes a message object of type '<DesiredSpeeds>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_speed))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DesiredSpeeds>) istream)
  "Deserializes a message object of type '<DesiredSpeeds>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_speed) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_speed)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DesiredSpeeds>)))
  "Returns string type for a message object of type '<DesiredSpeeds>"
  "kortex_driver/DesiredSpeeds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DesiredSpeeds)))
  "Returns string type for a message object of type 'DesiredSpeeds"
  "kortex_driver/DesiredSpeeds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DesiredSpeeds>)))
  "Returns md5sum for a message object of type '<DesiredSpeeds>"
  "9f61e0fe165a7f46b2508fd0832ff820")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DesiredSpeeds)))
  "Returns md5sum for a message object of type 'DesiredSpeeds"
  "9f61e0fe165a7f46b2508fd0832ff820")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DesiredSpeeds>)))
  "Returns full string definition for message of type '<DesiredSpeeds>"
  (cl:format cl:nil "~%float32 linear~%float32 angular~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DesiredSpeeds)))
  "Returns full string definition for message of type 'DesiredSpeeds"
  (cl:format cl:nil "~%float32 linear~%float32 angular~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DesiredSpeeds>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DesiredSpeeds>))
  "Converts a ROS message object to a list"
  (cl:list 'DesiredSpeeds
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
    (cl:cons ':joint_speed (joint_speed msg))
))
