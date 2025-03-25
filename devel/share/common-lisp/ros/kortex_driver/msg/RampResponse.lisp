; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RampResponse.msg.html

(cl:defclass <RampResponse> (roslisp-msg-protocol:ros-message)
  ((loop_selection
    :reader loop_selection
    :initarg :loop_selection
    :type cl:integer
    :initform 0)
   (slope
    :reader slope
    :initarg :slope
    :type cl:float
    :initform 0.0)
   (ramp_delay
    :reader ramp_delay
    :initarg :ramp_delay
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass RampResponse (<RampResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RampResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RampResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RampResponse> is deprecated: use kortex_driver-msg:RampResponse instead.")))

(cl:ensure-generic-function 'loop_selection-val :lambda-list '(m))
(cl:defmethod loop_selection-val ((m <RampResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:loop_selection-val is deprecated.  Use kortex_driver-msg:loop_selection instead.")
  (loop_selection m))

(cl:ensure-generic-function 'slope-val :lambda-list '(m))
(cl:defmethod slope-val ((m <RampResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:slope-val is deprecated.  Use kortex_driver-msg:slope instead.")
  (slope m))

(cl:ensure-generic-function 'ramp_delay-val :lambda-list '(m))
(cl:defmethod ramp_delay-val ((m <RampResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ramp_delay-val is deprecated.  Use kortex_driver-msg:ramp_delay instead.")
  (ramp_delay m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <RampResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RampResponse>) ostream)
  "Serializes a message object of type '<RampResponse>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ramp_delay))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RampResponse>) istream)
  "Deserializes a message object of type '<RampResponse>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'slope) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ramp_delay) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RampResponse>)))
  "Returns string type for a message object of type '<RampResponse>"
  "kortex_driver/RampResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RampResponse)))
  "Returns string type for a message object of type 'RampResponse"
  "kortex_driver/RampResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RampResponse>)))
  "Returns md5sum for a message object of type '<RampResponse>"
  "9e3f4edf9087b42bf3e2e7ff4ca2b2f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RampResponse)))
  "Returns md5sum for a message object of type 'RampResponse"
  "9e3f4edf9087b42bf3e2e7ff4ca2b2f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RampResponse>)))
  "Returns full string definition for message of type '<RampResponse>"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 slope~%float32 ramp_delay~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RampResponse)))
  "Returns full string definition for message of type 'RampResponse"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 slope~%float32 ramp_delay~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RampResponse>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RampResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'RampResponse
    (cl:cons ':loop_selection (loop_selection msg))
    (cl:cons ':slope (slope msg))
    (cl:cons ':ramp_delay (ramp_delay msg))
    (cl:cons ':duration (duration msg))
))
