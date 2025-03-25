; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude StepResponse.msg.html

(cl:defclass <StepResponse> (roslisp-msg-protocol:ros-message)
  ((loop_selection
    :reader loop_selection
    :initarg :loop_selection
    :type cl:integer
    :initform 0)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0)
   (step_delay
    :reader step_delay
    :initarg :step_delay
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass StepResponse (<StepResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StepResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StepResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<StepResponse> is deprecated: use kortex_driver-msg:StepResponse instead.")))

(cl:ensure-generic-function 'loop_selection-val :lambda-list '(m))
(cl:defmethod loop_selection-val ((m <StepResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:loop_selection-val is deprecated.  Use kortex_driver-msg:loop_selection instead.")
  (loop_selection m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <StepResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:amplitude-val is deprecated.  Use kortex_driver-msg:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'step_delay-val :lambda-list '(m))
(cl:defmethod step_delay-val ((m <StepResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:step_delay-val is deprecated.  Use kortex_driver-msg:step_delay instead.")
  (step_delay m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <StepResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StepResponse>) ostream)
  "Serializes a message object of type '<StepResponse>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'step_delay))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StepResponse>) istream)
  "Deserializes a message object of type '<StepResponse>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amplitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step_delay) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StepResponse>)))
  "Returns string type for a message object of type '<StepResponse>"
  "kortex_driver/StepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StepResponse)))
  "Returns string type for a message object of type 'StepResponse"
  "kortex_driver/StepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StepResponse>)))
  "Returns md5sum for a message object of type '<StepResponse>"
  "e9ae6749b10a5d88fae7f1a1e8639d93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StepResponse)))
  "Returns md5sum for a message object of type 'StepResponse"
  "e9ae6749b10a5d88fae7f1a1e8639d93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StepResponse>)))
  "Returns full string definition for message of type '<StepResponse>"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 amplitude~%float32 step_delay~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StepResponse)))
  "Returns full string definition for message of type 'StepResponse"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 amplitude~%float32 step_delay~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StepResponse>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StepResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'StepResponse
    (cl:cons ':loop_selection (loop_selection msg))
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':step_delay (step_delay msg))
    (cl:cons ':duration (duration msg))
))
