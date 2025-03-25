; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FrequencyResponse.msg.html

(cl:defclass <FrequencyResponse> (roslisp-msg-protocol:ros-message)
  ((loop_selection
    :reader loop_selection
    :initarg :loop_selection
    :type cl:integer
    :initform 0)
   (min_frequency
    :reader min_frequency
    :initarg :min_frequency
    :type cl:float
    :initform 0.0)
   (max_frequency
    :reader max_frequency
    :initarg :max_frequency
    :type cl:float
    :initform 0.0)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass FrequencyResponse (<FrequencyResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrequencyResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrequencyResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FrequencyResponse> is deprecated: use kortex_driver-msg:FrequencyResponse instead.")))

(cl:ensure-generic-function 'loop_selection-val :lambda-list '(m))
(cl:defmethod loop_selection-val ((m <FrequencyResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:loop_selection-val is deprecated.  Use kortex_driver-msg:loop_selection instead.")
  (loop_selection m))

(cl:ensure-generic-function 'min_frequency-val :lambda-list '(m))
(cl:defmethod min_frequency-val ((m <FrequencyResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:min_frequency-val is deprecated.  Use kortex_driver-msg:min_frequency instead.")
  (min_frequency m))

(cl:ensure-generic-function 'max_frequency-val :lambda-list '(m))
(cl:defmethod max_frequency-val ((m <FrequencyResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:max_frequency-val is deprecated.  Use kortex_driver-msg:max_frequency instead.")
  (max_frequency m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <FrequencyResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:amplitude-val is deprecated.  Use kortex_driver-msg:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <FrequencyResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrequencyResponse>) ostream)
  "Serializes a message object of type '<FrequencyResponse>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amplitude))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrequencyResponse>) istream)
  "Deserializes a message object of type '<FrequencyResponse>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_frequency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_frequency) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrequencyResponse>)))
  "Returns string type for a message object of type '<FrequencyResponse>"
  "kortex_driver/FrequencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrequencyResponse)))
  "Returns string type for a message object of type 'FrequencyResponse"
  "kortex_driver/FrequencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrequencyResponse>)))
  "Returns md5sum for a message object of type '<FrequencyResponse>"
  "86ed7a41c264d9e5ff7a52aa97222d1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrequencyResponse)))
  "Returns md5sum for a message object of type 'FrequencyResponse"
  "86ed7a41c264d9e5ff7a52aa97222d1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrequencyResponse>)))
  "Returns full string definition for message of type '<FrequencyResponse>"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 min_frequency~%float32 max_frequency~%float32 amplitude~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrequencyResponse)))
  "Returns full string definition for message of type 'FrequencyResponse"
  (cl:format cl:nil "~%uint32 loop_selection~%float32 min_frequency~%float32 max_frequency~%float32 amplitude~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrequencyResponse>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrequencyResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'FrequencyResponse
    (cl:cons ':loop_selection (loop_selection msg))
    (cl:cons ':min_frequency (min_frequency msg))
    (cl:cons ':max_frequency (max_frequency msg))
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':duration (duration msg))
))
