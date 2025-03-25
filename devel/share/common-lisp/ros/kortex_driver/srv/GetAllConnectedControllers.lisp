; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllConnectedControllers-request.msg.html

(cl:defclass <GetAllConnectedControllers-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllConnectedControllers-request (<GetAllConnectedControllers-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllConnectedControllers-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllConnectedControllers-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllConnectedControllers-request> is deprecated: use kortex_driver-srv:GetAllConnectedControllers-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllConnectedControllers-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllConnectedControllers-request>) ostream)
  "Serializes a message object of type '<GetAllConnectedControllers-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllConnectedControllers-request>) istream)
  "Deserializes a message object of type '<GetAllConnectedControllers-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllConnectedControllers-request>)))
  "Returns string type for a service object of type '<GetAllConnectedControllers-request>"
  "kortex_driver/GetAllConnectedControllersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllConnectedControllers-request)))
  "Returns string type for a service object of type 'GetAllConnectedControllers-request"
  "kortex_driver/GetAllConnectedControllersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllConnectedControllers-request>)))
  "Returns md5sum for a message object of type '<GetAllConnectedControllers-request>"
  "cf23ee5904b15aa6ac88d84a59fc18f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllConnectedControllers-request)))
  "Returns md5sum for a message object of type 'GetAllConnectedControllers-request"
  "cf23ee5904b15aa6ac88d84a59fc18f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllConnectedControllers-request>)))
  "Returns full string definition for message of type '<GetAllConnectedControllers-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllConnectedControllers-request)))
  "Returns full string definition for message of type 'GetAllConnectedControllers-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllConnectedControllers-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllConnectedControllers-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllConnectedControllers-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllConnectedControllers-response.msg.html

(cl:defclass <GetAllConnectedControllers-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControllerList
    :initform (cl:make-instance 'kortex_driver-msg:ControllerList)))
)

(cl:defclass GetAllConnectedControllers-response (<GetAllConnectedControllers-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllConnectedControllers-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllConnectedControllers-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllConnectedControllers-response> is deprecated: use kortex_driver-srv:GetAllConnectedControllers-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllConnectedControllers-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllConnectedControllers-response>) ostream)
  "Serializes a message object of type '<GetAllConnectedControllers-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllConnectedControllers-response>) istream)
  "Deserializes a message object of type '<GetAllConnectedControllers-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllConnectedControllers-response>)))
  "Returns string type for a service object of type '<GetAllConnectedControllers-response>"
  "kortex_driver/GetAllConnectedControllersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllConnectedControllers-response)))
  "Returns string type for a service object of type 'GetAllConnectedControllers-response"
  "kortex_driver/GetAllConnectedControllersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllConnectedControllers-response>)))
  "Returns md5sum for a message object of type '<GetAllConnectedControllers-response>"
  "cf23ee5904b15aa6ac88d84a59fc18f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllConnectedControllers-response)))
  "Returns md5sum for a message object of type 'GetAllConnectedControllers-response"
  "cf23ee5904b15aa6ac88d84a59fc18f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllConnectedControllers-response>)))
  "Returns full string definition for message of type '<GetAllConnectedControllers-response>"
  (cl:format cl:nil "ControllerList output~%~%================================================================================~%MSG: kortex_driver/ControllerList~%~%ControllerHandle[] handles~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllConnectedControllers-response)))
  "Returns full string definition for message of type 'GetAllConnectedControllers-response"
  (cl:format cl:nil "ControllerList output~%~%================================================================================~%MSG: kortex_driver/ControllerList~%~%ControllerHandle[] handles~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllConnectedControllers-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllConnectedControllers-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllConnectedControllers-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllConnectedControllers)))
  'GetAllConnectedControllers-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllConnectedControllers)))
  'GetAllConnectedControllers-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllConnectedControllers)))
  "Returns string type for a service object of type '<GetAllConnectedControllers>"
  "kortex_driver/GetAllConnectedControllers")