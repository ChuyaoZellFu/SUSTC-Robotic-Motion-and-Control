; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetProductConfiguration-request.msg.html

(cl:defclass <GetProductConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetProductConfiguration-request (<GetProductConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetProductConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetProductConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetProductConfiguration-request> is deprecated: use kortex_driver-srv:GetProductConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetProductConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetProductConfiguration-request>) ostream)
  "Serializes a message object of type '<GetProductConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetProductConfiguration-request>) istream)
  "Deserializes a message object of type '<GetProductConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetProductConfiguration-request>)))
  "Returns string type for a service object of type '<GetProductConfiguration-request>"
  "kortex_driver/GetProductConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetProductConfiguration-request)))
  "Returns string type for a service object of type 'GetProductConfiguration-request"
  "kortex_driver/GetProductConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetProductConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetProductConfiguration-request>"
  "899fbdc53f9306591e652c7c52962785")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetProductConfiguration-request)))
  "Returns md5sum for a message object of type 'GetProductConfiguration-request"
  "899fbdc53f9306591e652c7c52962785")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetProductConfiguration-request>)))
  "Returns full string definition for message of type '<GetProductConfiguration-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetProductConfiguration-request)))
  "Returns full string definition for message of type 'GetProductConfiguration-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetProductConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetProductConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetProductConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetProductConfiguration-response.msg.html

(cl:defclass <GetProductConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CompleteProductConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:CompleteProductConfiguration)))
)

(cl:defclass GetProductConfiguration-response (<GetProductConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetProductConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetProductConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetProductConfiguration-response> is deprecated: use kortex_driver-srv:GetProductConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetProductConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetProductConfiguration-response>) ostream)
  "Serializes a message object of type '<GetProductConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetProductConfiguration-response>) istream)
  "Deserializes a message object of type '<GetProductConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetProductConfiguration-response>)))
  "Returns string type for a service object of type '<GetProductConfiguration-response>"
  "kortex_driver/GetProductConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetProductConfiguration-response)))
  "Returns string type for a service object of type 'GetProductConfiguration-response"
  "kortex_driver/GetProductConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetProductConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetProductConfiguration-response>"
  "899fbdc53f9306591e652c7c52962785")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetProductConfiguration-response)))
  "Returns md5sum for a message object of type 'GetProductConfiguration-response"
  "899fbdc53f9306591e652c7c52962785")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetProductConfiguration-response>)))
  "Returns full string definition for message of type '<GetProductConfiguration-response>"
  (cl:format cl:nil "CompleteProductConfiguration output~%~%================================================================================~%MSG: kortex_driver/CompleteProductConfiguration~%~%string kin~%uint32 model~%CountryCode country_code~%string assembly_plant~%string model_year~%uint32 degree_of_freedom~%uint32 base_type~%uint32 end_effector_type~%uint32 vision_module_type~%uint32 interface_module_type~%uint32 arm_laterality~%uint32 wrist_type~%uint32 brake_type~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetProductConfiguration-response)))
  "Returns full string definition for message of type 'GetProductConfiguration-response"
  (cl:format cl:nil "CompleteProductConfiguration output~%~%================================================================================~%MSG: kortex_driver/CompleteProductConfiguration~%~%string kin~%uint32 model~%CountryCode country_code~%string assembly_plant~%string model_year~%uint32 degree_of_freedom~%uint32 base_type~%uint32 end_effector_type~%uint32 vision_module_type~%uint32 interface_module_type~%uint32 arm_laterality~%uint32 wrist_type~%uint32 brake_type~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetProductConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetProductConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetProductConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetProductConfiguration)))
  'GetProductConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetProductConfiguration)))
  'GetProductConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetProductConfiguration)))
  "Returns string type for a service object of type '<GetProductConfiguration>"
  "kortex_driver/GetProductConfiguration")