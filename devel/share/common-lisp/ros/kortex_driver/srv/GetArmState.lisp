; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetArmState-request.msg.html

(cl:defclass <GetArmState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetArmState-request (<GetArmState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetArmState-request> is deprecated: use kortex_driver-srv:GetArmState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetArmState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmState-request>) ostream)
  "Serializes a message object of type '<GetArmState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmState-request>) istream)
  "Deserializes a message object of type '<GetArmState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmState-request>)))
  "Returns string type for a service object of type '<GetArmState-request>"
  "kortex_driver/GetArmStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmState-request)))
  "Returns string type for a service object of type 'GetArmState-request"
  "kortex_driver/GetArmStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmState-request>)))
  "Returns md5sum for a message object of type '<GetArmState-request>"
  "09192f2d3f6462e1466a4468f2c2390a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmState-request)))
  "Returns md5sum for a message object of type 'GetArmState-request"
  "09192f2d3f6462e1466a4468f2c2390a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmState-request>)))
  "Returns full string definition for message of type '<GetArmState-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmState-request)))
  "Returns full string definition for message of type 'GetArmState-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetArmState-response.msg.html

(cl:defclass <GetArmState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ArmStateInformation
    :initform (cl:make-instance 'kortex_driver-msg:ArmStateInformation)))
)

(cl:defclass GetArmState-response (<GetArmState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetArmState-response> is deprecated: use kortex_driver-srv:GetArmState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetArmState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmState-response>) ostream)
  "Serializes a message object of type '<GetArmState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmState-response>) istream)
  "Deserializes a message object of type '<GetArmState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmState-response>)))
  "Returns string type for a service object of type '<GetArmState-response>"
  "kortex_driver/GetArmStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmState-response)))
  "Returns string type for a service object of type 'GetArmState-response"
  "kortex_driver/GetArmStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmState-response>)))
  "Returns md5sum for a message object of type '<GetArmState-response>"
  "09192f2d3f6462e1466a4468f2c2390a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmState-response)))
  "Returns md5sum for a message object of type 'GetArmState-response"
  "09192f2d3f6462e1466a4468f2c2390a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmState-response>)))
  "Returns full string definition for message of type '<GetArmState-response>"
  (cl:format cl:nil "ArmStateInformation output~%~%================================================================================~%MSG: kortex_driver/ArmStateInformation~%~%uint32 active_state~%Connection connection~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmState-response)))
  "Returns full string definition for message of type 'GetArmState-response"
  (cl:format cl:nil "ArmStateInformation output~%~%================================================================================~%MSG: kortex_driver/ArmStateInformation~%~%uint32 active_state~%Connection connection~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetArmState)))
  'GetArmState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetArmState)))
  'GetArmState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmState)))
  "Returns string type for a service object of type '<GetArmState>"
  "kortex_driver/GetArmState")