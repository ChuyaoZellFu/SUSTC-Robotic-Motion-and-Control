; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MapEvent_events.msg.html

(cl:defclass <MapEvent_events> (roslisp-msg-protocol:ros-message)
  ((safety_event
    :reader safety_event
    :initarg :safety_event
    :type (cl:vector kortex_driver-msg:SafetyEvent)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:SafetyEvent :initial-element (cl:make-instance 'kortex_driver-msg:SafetyEvent)))
   (gpio_event
    :reader gpio_event
    :initarg :gpio_event
    :type (cl:vector kortex_driver-msg:GpioEvent)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:GpioEvent :initial-element (cl:make-instance 'kortex_driver-msg:GpioEvent)))
   (controller_event
    :reader controller_event
    :initarg :controller_event
    :type (cl:vector kortex_driver-msg:ControllerEvent)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ControllerEvent :initial-element (cl:make-instance 'kortex_driver-msg:ControllerEvent))))
)

(cl:defclass MapEvent_events (<MapEvent_events>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEvent_events>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEvent_events)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MapEvent_events> is deprecated: use kortex_driver-msg:MapEvent_events instead.")))

(cl:ensure-generic-function 'safety_event-val :lambda-list '(m))
(cl:defmethod safety_event-val ((m <MapEvent_events>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:safety_event-val is deprecated.  Use kortex_driver-msg:safety_event instead.")
  (safety_event m))

(cl:ensure-generic-function 'gpio_event-val :lambda-list '(m))
(cl:defmethod gpio_event-val ((m <MapEvent_events>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:gpio_event-val is deprecated.  Use kortex_driver-msg:gpio_event instead.")
  (gpio_event m))

(cl:ensure-generic-function 'controller_event-val :lambda-list '(m))
(cl:defmethod controller_event-val ((m <MapEvent_events>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_event-val is deprecated.  Use kortex_driver-msg:controller_event instead.")
  (controller_event m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEvent_events>) ostream)
  "Serializes a message object of type '<MapEvent_events>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'safety_event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'safety_event))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gpio_event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gpio_event))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controller_event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controller_event))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEvent_events>) istream)
  "Deserializes a message object of type '<MapEvent_events>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'safety_event) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'safety_event)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:SafetyEvent))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gpio_event) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gpio_event)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:GpioEvent))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controller_event) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controller_event)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ControllerEvent))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEvent_events>)))
  "Returns string type for a message object of type '<MapEvent_events>"
  "kortex_driver/MapEvent_events")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEvent_events)))
  "Returns string type for a message object of type 'MapEvent_events"
  "kortex_driver/MapEvent_events")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEvent_events>)))
  "Returns md5sum for a message object of type '<MapEvent_events>"
  "040caa44c991691249bac9565abbf70c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEvent_events)))
  "Returns md5sum for a message object of type 'MapEvent_events"
  "040caa44c991691249bac9565abbf70c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEvent_events>)))
  "Returns full string definition for message of type '<MapEvent_events>"
  (cl:format cl:nil "~%SafetyEvent[] safety_event~%GpioEvent[] gpio_event~%ControllerEvent[] controller_event~%================================================================================~%MSG: kortex_driver/SafetyEvent~%~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/GpioEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/ControllerEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEvent_events)))
  "Returns full string definition for message of type 'MapEvent_events"
  (cl:format cl:nil "~%SafetyEvent[] safety_event~%GpioEvent[] gpio_event~%ControllerEvent[] controller_event~%================================================================================~%MSG: kortex_driver/SafetyEvent~%~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/GpioEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/ControllerEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEvent_events>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'safety_event) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gpio_event) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controller_event) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEvent_events>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEvent_events
    (cl:cons ':safety_event (safety_event msg))
    (cl:cons ':gpio_event (gpio_event msg))
    (cl:cons ':controller_event (controller_event msg))
))
