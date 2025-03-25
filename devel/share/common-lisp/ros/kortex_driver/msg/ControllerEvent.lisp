; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerEvent.msg.html

(cl:defclass <ControllerEvent> (roslisp-msg-protocol:ros-message)
  ((input_type
    :reader input_type
    :initarg :input_type
    :type cl:integer
    :initform 0)
   (behavior
    :reader behavior
    :initarg :behavior
    :type cl:integer
    :initform 0)
   (input_identifier
    :reader input_identifier
    :initarg :input_identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass ControllerEvent (<ControllerEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerEvent> is deprecated: use kortex_driver-msg:ControllerEvent instead.")))

(cl:ensure-generic-function 'input_type-val :lambda-list '(m))
(cl:defmethod input_type-val ((m <ControllerEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:input_type-val is deprecated.  Use kortex_driver-msg:input_type instead.")
  (input_type m))

(cl:ensure-generic-function 'behavior-val :lambda-list '(m))
(cl:defmethod behavior-val ((m <ControllerEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:behavior-val is deprecated.  Use kortex_driver-msg:behavior instead.")
  (behavior m))

(cl:ensure-generic-function 'input_identifier-val :lambda-list '(m))
(cl:defmethod input_identifier-val ((m <ControllerEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:input_identifier-val is deprecated.  Use kortex_driver-msg:input_identifier instead.")
  (input_identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerEvent>) ostream)
  "Serializes a message object of type '<ControllerEvent>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'input_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'input_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'input_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'input_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'behavior)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'behavior)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'behavior)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'behavior)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'input_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'input_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'input_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'input_identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerEvent>) istream)
  "Deserializes a message object of type '<ControllerEvent>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'input_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'input_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'input_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'input_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'behavior)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'behavior)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'behavior)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'behavior)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'input_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'input_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'input_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'input_identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerEvent>)))
  "Returns string type for a message object of type '<ControllerEvent>"
  "kortex_driver/ControllerEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerEvent)))
  "Returns string type for a message object of type 'ControllerEvent"
  "kortex_driver/ControllerEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerEvent>)))
  "Returns md5sum for a message object of type '<ControllerEvent>"
  "a8f7ada35fd120a9401b9d95b206763c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerEvent)))
  "Returns md5sum for a message object of type 'ControllerEvent"
  "a8f7ada35fd120a9401b9d95b206763c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerEvent>)))
  "Returns full string definition for message of type '<ControllerEvent>"
  (cl:format cl:nil "~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerEvent)))
  "Returns full string definition for message of type 'ControllerEvent"
  (cl:format cl:nil "~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerEvent>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerEvent
    (cl:cons ':input_type (input_type msg))
    (cl:cons ':behavior (behavior msg))
    (cl:cons ':input_identifier (input_identifier msg))
))
