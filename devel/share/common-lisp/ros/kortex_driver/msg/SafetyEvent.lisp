; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyEvent.msg.html

(cl:defclass <SafetyEvent> (roslisp-msg-protocol:ros-message)
  ((safety_handle
    :reader safety_handle
    :initarg :safety_handle
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle)))
)

(cl:defclass SafetyEvent (<SafetyEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyEvent> is deprecated: use kortex_driver-msg:SafetyEvent instead.")))

(cl:ensure-generic-function 'safety_handle-val :lambda-list '(m))
(cl:defmethod safety_handle-val ((m <SafetyEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:safety_handle-val is deprecated.  Use kortex_driver-msg:safety_handle instead.")
  (safety_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyEvent>) ostream)
  "Serializes a message object of type '<SafetyEvent>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'safety_handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyEvent>) istream)
  "Deserializes a message object of type '<SafetyEvent>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'safety_handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyEvent>)))
  "Returns string type for a message object of type '<SafetyEvent>"
  "kortex_driver/SafetyEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyEvent)))
  "Returns string type for a message object of type 'SafetyEvent"
  "kortex_driver/SafetyEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyEvent>)))
  "Returns md5sum for a message object of type '<SafetyEvent>"
  "48c102b68919349f9a9bb31c7e295748")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyEvent)))
  "Returns md5sum for a message object of type 'SafetyEvent"
  "48c102b68919349f9a9bb31c7e295748")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyEvent>)))
  "Returns full string definition for message of type '<SafetyEvent>"
  (cl:format cl:nil "~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyEvent)))
  "Returns full string definition for message of type 'SafetyEvent"
  (cl:format cl:nil "~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyEvent>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'safety_handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyEvent
    (cl:cons ':safety_handle (safety_handle msg))
))
