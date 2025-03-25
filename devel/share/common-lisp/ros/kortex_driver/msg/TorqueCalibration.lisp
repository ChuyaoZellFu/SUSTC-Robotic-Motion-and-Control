; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TorqueCalibration.msg.html

(cl:defclass <TorqueCalibration> (roslisp-msg-protocol:ros-message)
  ((global_gain
    :reader global_gain
    :initarg :global_gain
    :type cl:float
    :initform 0.0)
   (global_offset
    :reader global_offset
    :initarg :global_offset
    :type cl:float
    :initform 0.0)
   (gain
    :reader gain
    :initarg :gain
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (offset
    :reader offset
    :initarg :offset
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass TorqueCalibration (<TorqueCalibration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorqueCalibration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorqueCalibration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TorqueCalibration> is deprecated: use kortex_driver-msg:TorqueCalibration instead.")))

(cl:ensure-generic-function 'global_gain-val :lambda-list '(m))
(cl:defmethod global_gain-val ((m <TorqueCalibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:global_gain-val is deprecated.  Use kortex_driver-msg:global_gain instead.")
  (global_gain m))

(cl:ensure-generic-function 'global_offset-val :lambda-list '(m))
(cl:defmethod global_offset-val ((m <TorqueCalibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:global_offset-val is deprecated.  Use kortex_driver-msg:global_offset instead.")
  (global_offset m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <TorqueCalibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:gain-val is deprecated.  Use kortex_driver-msg:gain instead.")
  (gain m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <TorqueCalibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:offset-val is deprecated.  Use kortex_driver-msg:offset instead.")
  (offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorqueCalibration>) ostream)
  "Serializes a message object of type '<TorqueCalibration>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'global_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'global_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'gain))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'offset))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorqueCalibration>) istream)
  "Deserializes a message object of type '<TorqueCalibration>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'global_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'global_offset) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gain) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gain)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'offset) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'offset)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorqueCalibration>)))
  "Returns string type for a message object of type '<TorqueCalibration>"
  "kortex_driver/TorqueCalibration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorqueCalibration)))
  "Returns string type for a message object of type 'TorqueCalibration"
  "kortex_driver/TorqueCalibration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorqueCalibration>)))
  "Returns md5sum for a message object of type '<TorqueCalibration>"
  "313d945f71d43fe1540908829effe904")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorqueCalibration)))
  "Returns md5sum for a message object of type 'TorqueCalibration"
  "313d945f71d43fe1540908829effe904")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorqueCalibration>)))
  "Returns full string definition for message of type '<TorqueCalibration>"
  (cl:format cl:nil "~%float32 global_gain~%float32 global_offset~%float32[] gain~%float32[] offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorqueCalibration)))
  "Returns full string definition for message of type 'TorqueCalibration"
  (cl:format cl:nil "~%float32 global_gain~%float32 global_offset~%float32[] gain~%float32[] offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorqueCalibration>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gain) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'offset) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorqueCalibration>))
  "Converts a ROS message object to a list"
  (cl:list 'TorqueCalibration
    (cl:cons ':global_gain (global_gain msg))
    (cl:cons ':global_offset (global_offset msg))
    (cl:cons ':gain (gain msg))
    (cl:cons ':offset (offset msg))
))
