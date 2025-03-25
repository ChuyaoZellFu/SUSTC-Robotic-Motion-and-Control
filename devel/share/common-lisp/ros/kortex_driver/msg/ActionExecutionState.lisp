; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActionExecutionState.msg.html

(cl:defclass <ActionExecutionState> (roslisp-msg-protocol:ros-message)
  ((action_event
    :reader action_event
    :initarg :action_event
    :type cl:integer
    :initform 0)
   (handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:ActionHandle
    :initform (cl:make-instance 'kortex_driver-msg:ActionHandle)))
)

(cl:defclass ActionExecutionState (<ActionExecutionState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionExecutionState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionExecutionState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActionExecutionState> is deprecated: use kortex_driver-msg:ActionExecutionState instead.")))

(cl:ensure-generic-function 'action_event-val :lambda-list '(m))
(cl:defmethod action_event-val ((m <ActionExecutionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:action_event-val is deprecated.  Use kortex_driver-msg:action_event instead.")
  (action_event m))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <ActionExecutionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionExecutionState>) ostream)
  "Serializes a message object of type '<ActionExecutionState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action_event)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionExecutionState>) istream)
  "Deserializes a message object of type '<ActionExecutionState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action_event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action_event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action_event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action_event)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionExecutionState>)))
  "Returns string type for a message object of type '<ActionExecutionState>"
  "kortex_driver/ActionExecutionState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionExecutionState)))
  "Returns string type for a message object of type 'ActionExecutionState"
  "kortex_driver/ActionExecutionState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionExecutionState>)))
  "Returns md5sum for a message object of type '<ActionExecutionState>"
  "ffbf0e1fde1af8a50b9f2f937e17d3ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionExecutionState)))
  "Returns md5sum for a message object of type 'ActionExecutionState"
  "ffbf0e1fde1af8a50b9f2f937e17d3ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionExecutionState>)))
  "Returns full string definition for message of type '<ActionExecutionState>"
  (cl:format cl:nil "~%uint32 action_event~%ActionHandle handle~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionExecutionState)))
  "Returns full string definition for message of type 'ActionExecutionState"
  (cl:format cl:nil "~%uint32 action_event~%ActionHandle handle~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionExecutionState>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionExecutionState>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionExecutionState
    (cl:cons ':action_event (action_event msg))
    (cl:cons ':handle (handle msg))
))
