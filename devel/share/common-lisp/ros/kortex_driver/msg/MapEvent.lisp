; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MapEvent.msg.html

(cl:defclass <MapEvent> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (oneof_events
    :reader oneof_events
    :initarg :oneof_events
    :type kortex_driver-msg:MapEvent_events
    :initform (cl:make-instance 'kortex_driver-msg:MapEvent_events)))
)

(cl:defclass MapEvent (<MapEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MapEvent> is deprecated: use kortex_driver-msg:MapEvent instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MapEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'oneof_events-val :lambda-list '(m))
(cl:defmethod oneof_events-val ((m <MapEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_events-val is deprecated.  Use kortex_driver-msg:oneof_events instead.")
  (oneof_events m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEvent>) ostream)
  "Serializes a message object of type '<MapEvent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_events) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEvent>) istream)
  "Deserializes a message object of type '<MapEvent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_events) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEvent>)))
  "Returns string type for a message object of type '<MapEvent>"
  "kortex_driver/MapEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEvent)))
  "Returns string type for a message object of type 'MapEvent"
  "kortex_driver/MapEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEvent>)))
  "Returns md5sum for a message object of type '<MapEvent>"
  "c2332688b9b21902d710a4725da3d8c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEvent)))
  "Returns md5sum for a message object of type 'MapEvent"
  "c2332688b9b21902d710a4725da3d8c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEvent>)))
  "Returns full string definition for message of type '<MapEvent>"
  (cl:format cl:nil "~%string name~%MapEvent_events oneof_events~%================================================================================~%MSG: kortex_driver/MapEvent_events~%~%SafetyEvent[] safety_event~%GpioEvent[] gpio_event~%ControllerEvent[] controller_event~%================================================================================~%MSG: kortex_driver/SafetyEvent~%~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/GpioEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/ControllerEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEvent)))
  "Returns full string definition for message of type 'MapEvent"
  (cl:format cl:nil "~%string name~%MapEvent_events oneof_events~%================================================================================~%MSG: kortex_driver/MapEvent_events~%~%SafetyEvent[] safety_event~%GpioEvent[] gpio_event~%ControllerEvent[] controller_event~%================================================================================~%MSG: kortex_driver/SafetyEvent~%~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/GpioEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/ControllerEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEvent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_events))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEvent
    (cl:cons ':name (name msg))
    (cl:cons ':oneof_events (oneof_events msg))
))
