; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerNotification_state.msg.html

(cl:defclass <ControllerNotification_state> (roslisp-msg-protocol:ros-message)
  ((controller_state
    :reader controller_state
    :initarg :controller_state
    :type (cl:vector kortex_driver-msg:ControllerState)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ControllerState :initial-element (cl:make-instance 'kortex_driver-msg:ControllerState)))
   (controller_element
    :reader controller_element
    :initarg :controller_element
    :type (cl:vector kortex_driver-msg:ControllerElementState)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ControllerElementState :initial-element (cl:make-instance 'kortex_driver-msg:ControllerElementState))))
)

(cl:defclass ControllerNotification_state (<ControllerNotification_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerNotification_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerNotification_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerNotification_state> is deprecated: use kortex_driver-msg:ControllerNotification_state instead.")))

(cl:ensure-generic-function 'controller_state-val :lambda-list '(m))
(cl:defmethod controller_state-val ((m <ControllerNotification_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_state-val is deprecated.  Use kortex_driver-msg:controller_state instead.")
  (controller_state m))

(cl:ensure-generic-function 'controller_element-val :lambda-list '(m))
(cl:defmethod controller_element-val ((m <ControllerNotification_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_element-val is deprecated.  Use kortex_driver-msg:controller_element instead.")
  (controller_element m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerNotification_state>) ostream)
  "Serializes a message object of type '<ControllerNotification_state>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controller_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controller_state))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controller_element))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controller_element))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerNotification_state>) istream)
  "Deserializes a message object of type '<ControllerNotification_state>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controller_state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controller_state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ControllerState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controller_element) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controller_element)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ControllerElementState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerNotification_state>)))
  "Returns string type for a message object of type '<ControllerNotification_state>"
  "kortex_driver/ControllerNotification_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerNotification_state)))
  "Returns string type for a message object of type 'ControllerNotification_state"
  "kortex_driver/ControllerNotification_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerNotification_state>)))
  "Returns md5sum for a message object of type '<ControllerNotification_state>"
  "f624e32135515547aa37c46203c9be9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerNotification_state)))
  "Returns md5sum for a message object of type 'ControllerNotification_state"
  "f624e32135515547aa37c46203c9be9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerNotification_state>)))
  "Returns full string definition for message of type '<ControllerNotification_state>"
  (cl:format cl:nil "~%ControllerState[] controller_state~%ControllerElementState[] controller_element~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementState~%~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerNotification_state)))
  "Returns full string definition for message of type 'ControllerNotification_state"
  (cl:format cl:nil "~%ControllerState[] controller_state~%ControllerElementState[] controller_element~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementState~%~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerNotification_state>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controller_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controller_element) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerNotification_state>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerNotification_state
    (cl:cons ':controller_state (controller_state msg))
    (cl:cons ':controller_element (controller_element msg))
))
