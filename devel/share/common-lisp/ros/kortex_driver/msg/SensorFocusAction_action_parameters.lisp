; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SensorFocusAction_action_parameters.msg.html

(cl:defclass <SensorFocusAction_action_parameters> (roslisp-msg-protocol:ros-message)
  ((focus_point
    :reader focus_point
    :initarg :focus_point
    :type (cl:vector kortex_driver-msg:FocusPoint)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:FocusPoint :initial-element (cl:make-instance 'kortex_driver-msg:FocusPoint)))
   (manual_focus
    :reader manual_focus
    :initarg :manual_focus
    :type (cl:vector kortex_driver-msg:ManualFocus)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ManualFocus :initial-element (cl:make-instance 'kortex_driver-msg:ManualFocus))))
)

(cl:defclass SensorFocusAction_action_parameters (<SensorFocusAction_action_parameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorFocusAction_action_parameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorFocusAction_action_parameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SensorFocusAction_action_parameters> is deprecated: use kortex_driver-msg:SensorFocusAction_action_parameters instead.")))

(cl:ensure-generic-function 'focus_point-val :lambda-list '(m))
(cl:defmethod focus_point-val ((m <SensorFocusAction_action_parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:focus_point-val is deprecated.  Use kortex_driver-msg:focus_point instead.")
  (focus_point m))

(cl:ensure-generic-function 'manual_focus-val :lambda-list '(m))
(cl:defmethod manual_focus-val ((m <SensorFocusAction_action_parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:manual_focus-val is deprecated.  Use kortex_driver-msg:manual_focus instead.")
  (manual_focus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorFocusAction_action_parameters>) ostream)
  "Serializes a message object of type '<SensorFocusAction_action_parameters>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'focus_point))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'focus_point))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'manual_focus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'manual_focus))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorFocusAction_action_parameters>) istream)
  "Deserializes a message object of type '<SensorFocusAction_action_parameters>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'focus_point) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'focus_point)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:FocusPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'manual_focus) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'manual_focus)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ManualFocus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorFocusAction_action_parameters>)))
  "Returns string type for a message object of type '<SensorFocusAction_action_parameters>"
  "kortex_driver/SensorFocusAction_action_parameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorFocusAction_action_parameters)))
  "Returns string type for a message object of type 'SensorFocusAction_action_parameters"
  "kortex_driver/SensorFocusAction_action_parameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorFocusAction_action_parameters>)))
  "Returns md5sum for a message object of type '<SensorFocusAction_action_parameters>"
  "6e07eb3126a69325e87d9841916b8dc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorFocusAction_action_parameters)))
  "Returns md5sum for a message object of type 'SensorFocusAction_action_parameters"
  "6e07eb3126a69325e87d9841916b8dc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorFocusAction_action_parameters>)))
  "Returns full string definition for message of type '<SensorFocusAction_action_parameters>"
  (cl:format cl:nil "~%FocusPoint[] focus_point~%ManualFocus[] manual_focus~%================================================================================~%MSG: kortex_driver/FocusPoint~%~%uint32 x~%uint32 y~%================================================================================~%MSG: kortex_driver/ManualFocus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorFocusAction_action_parameters)))
  "Returns full string definition for message of type 'SensorFocusAction_action_parameters"
  (cl:format cl:nil "~%FocusPoint[] focus_point~%ManualFocus[] manual_focus~%================================================================================~%MSG: kortex_driver/FocusPoint~%~%uint32 x~%uint32 y~%================================================================================~%MSG: kortex_driver/ManualFocus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorFocusAction_action_parameters>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'focus_point) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'manual_focus) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorFocusAction_action_parameters>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorFocusAction_action_parameters
    (cl:cons ':focus_point (focus_point msg))
    (cl:cons ':manual_focus (manual_focus msg))
))
