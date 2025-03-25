; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlLoopParameters.msg.html

(cl:defclass <ControlLoopParameters> (roslisp-msg-protocol:ros-message)
  ((loop_selection
    :reader loop_selection
    :initarg :loop_selection
    :type cl:integer
    :initform 0)
   (error_saturation
    :reader error_saturation
    :initarg :error_saturation
    :type cl:float
    :initform 0.0)
   (output_saturation
    :reader output_saturation
    :initarg :output_saturation
    :type cl:float
    :initform 0.0)
   (kAz
    :reader kAz
    :initarg :kAz
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (kBz
    :reader kBz
    :initarg :kBz
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (error_dead_band
    :reader error_dead_band
    :initarg :error_dead_band
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlLoopParameters (<ControlLoopParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlLoopParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlLoopParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlLoopParameters> is deprecated: use kortex_driver-msg:ControlLoopParameters instead.")))

(cl:ensure-generic-function 'loop_selection-val :lambda-list '(m))
(cl:defmethod loop_selection-val ((m <ControlLoopParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:loop_selection-val is deprecated.  Use kortex_driver-msg:loop_selection instead.")
  (loop_selection m))

(cl:ensure-generic-function 'error_saturation-val :lambda-list '(m))
(cl:defmethod error_saturation-val ((m <ControlLoopParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:error_saturation-val is deprecated.  Use kortex_driver-msg:error_saturation instead.")
  (error_saturation m))

(cl:ensure-generic-function 'output_saturation-val :lambda-list '(m))
(cl:defmethod output_saturation-val ((m <ControlLoopParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:output_saturation-val is deprecated.  Use kortex_driver-msg:output_saturation instead.")
  (output_saturation m))

(cl:ensure-generic-function 'kAz-val :lambda-list '(m))
(cl:defmethod kAz-val ((m <ControlLoopParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kAz-val is deprecated.  Use kortex_driver-msg:kAz instead.")
  (kAz m))

(cl:ensure-generic-function 'kBz-val :lambda-list '(m))
(cl:defmethod kBz-val ((m <ControlLoopParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kBz-val is deprecated.  Use kortex_driver-msg:kBz instead.")
  (kBz m))

(cl:ensure-generic-function 'error_dead_band-val :lambda-list '(m))
(cl:defmethod error_dead_band-val ((m <ControlLoopParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:error_dead_band-val is deprecated.  Use kortex_driver-msg:error_dead_band instead.")
  (error_dead_band m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlLoopParameters>) ostream)
  "Serializes a message object of type '<ControlLoopParameters>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_saturation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output_saturation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'kAz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'kAz))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'kBz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'kBz))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_dead_band))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlLoopParameters>) istream)
  "Deserializes a message object of type '<ControlLoopParameters>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_saturation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output_saturation) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'kAz) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'kAz)))
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
  (cl:setf (cl:slot-value msg 'kBz) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'kBz)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_dead_band) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlLoopParameters>)))
  "Returns string type for a message object of type '<ControlLoopParameters>"
  "kortex_driver/ControlLoopParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlLoopParameters)))
  "Returns string type for a message object of type 'ControlLoopParameters"
  "kortex_driver/ControlLoopParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlLoopParameters>)))
  "Returns md5sum for a message object of type '<ControlLoopParameters>"
  "309e14eef078d5bea86d94317d3d0e04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlLoopParameters)))
  "Returns md5sum for a message object of type 'ControlLoopParameters"
  "309e14eef078d5bea86d94317d3d0e04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlLoopParameters>)))
  "Returns full string definition for message of type '<ControlLoopParameters>"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 error_saturation~%float32 output_saturation~%float32[] kAz~%float32[] kBz~%float32 error_dead_band~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlLoopParameters)))
  "Returns full string definition for message of type 'ControlLoopParameters"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 error_saturation~%float32 output_saturation~%float32[] kAz~%float32[] kBz~%float32 error_dead_band~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlLoopParameters>))
  (cl:+ 0
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'kAz) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'kBz) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlLoopParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlLoopParameters
    (cl:cons ':loop_selection (loop_selection msg))
    (cl:cons ':error_saturation (error_saturation msg))
    (cl:cons ':output_saturation (output_saturation msg))
    (cl:cons ':kAz (kAz msg))
    (cl:cons ':kBz (kBz msg))
    (cl:cons ':error_dead_band (error_dead_band msg))
))
