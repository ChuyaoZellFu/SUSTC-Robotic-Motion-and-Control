; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerElementState.msg.html

(cl:defclass <ControllerElementState> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:ControllerElementHandle
    :initform (cl:make-instance 'kortex_driver-msg:ControllerElementHandle))
   (event_type
    :reader event_type
    :initarg :event_type
    :type cl:integer
    :initform 0)
   (axis_value
    :reader axis_value
    :initarg :axis_value
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControllerElementState (<ControllerElementState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerElementState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerElementState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerElementState> is deprecated: use kortex_driver-msg:ControllerElementState instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <ControllerElementState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'event_type-val :lambda-list '(m))
(cl:defmethod event_type-val ((m <ControllerElementState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event_type-val is deprecated.  Use kortex_driver-msg:event_type instead.")
  (event_type m))

(cl:ensure-generic-function 'axis_value-val :lambda-list '(m))
(cl:defmethod axis_value-val ((m <ControllerElementState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:axis_value-val is deprecated.  Use kortex_driver-msg:axis_value instead.")
  (axis_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerElementState>) ostream)
  "Serializes a message object of type '<ControllerElementState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event_type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'axis_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerElementState>) istream)
  "Deserializes a message object of type '<ControllerElementState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event_type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'axis_value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerElementState>)))
  "Returns string type for a message object of type '<ControllerElementState>"
  "kortex_driver/ControllerElementState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerElementState)))
  "Returns string type for a message object of type 'ControllerElementState"
  "kortex_driver/ControllerElementState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerElementState>)))
  "Returns md5sum for a message object of type '<ControllerElementState>"
  "522b5634710ce23afc015e933c822f1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerElementState)))
  "Returns md5sum for a message object of type 'ControllerElementState"
  "522b5634710ce23afc015e933c822f1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerElementState>)))
  "Returns full string definition for message of type '<ControllerElementState>"
  (cl:format cl:nil "~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerElementState)))
  "Returns full string definition for message of type 'ControllerElementState"
  (cl:format cl:nil "~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerElementState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerElementState>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerElementState
    (cl:cons ':handle (handle msg))
    (cl:cons ':event_type (event_type msg))
    (cl:cons ':axis_value (axis_value msg))
))
