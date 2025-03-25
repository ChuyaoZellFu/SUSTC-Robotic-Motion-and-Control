; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteWifiConfiguration-request.msg.html

(cl:defclass <DeleteWifiConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Ssid
    :initform (cl:make-instance 'kortex_driver-msg:Ssid)))
)

(cl:defclass DeleteWifiConfiguration-request (<DeleteWifiConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteWifiConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteWifiConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteWifiConfiguration-request> is deprecated: use kortex_driver-srv:DeleteWifiConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteWifiConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteWifiConfiguration-request>) ostream)
  "Serializes a message object of type '<DeleteWifiConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteWifiConfiguration-request>) istream)
  "Deserializes a message object of type '<DeleteWifiConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteWifiConfiguration-request>)))
  "Returns string type for a service object of type '<DeleteWifiConfiguration-request>"
  "kortex_driver/DeleteWifiConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteWifiConfiguration-request)))
  "Returns string type for a service object of type 'DeleteWifiConfiguration-request"
  "kortex_driver/DeleteWifiConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteWifiConfiguration-request>)))
  "Returns md5sum for a message object of type '<DeleteWifiConfiguration-request>"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteWifiConfiguration-request)))
  "Returns md5sum for a message object of type 'DeleteWifiConfiguration-request"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteWifiConfiguration-request>)))
  "Returns full string definition for message of type '<DeleteWifiConfiguration-request>"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteWifiConfiguration-request)))
  "Returns full string definition for message of type 'DeleteWifiConfiguration-request"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteWifiConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteWifiConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteWifiConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteWifiConfiguration-response.msg.html

(cl:defclass <DeleteWifiConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteWifiConfiguration-response (<DeleteWifiConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteWifiConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteWifiConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteWifiConfiguration-response> is deprecated: use kortex_driver-srv:DeleteWifiConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteWifiConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteWifiConfiguration-response>) ostream)
  "Serializes a message object of type '<DeleteWifiConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteWifiConfiguration-response>) istream)
  "Deserializes a message object of type '<DeleteWifiConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteWifiConfiguration-response>)))
  "Returns string type for a service object of type '<DeleteWifiConfiguration-response>"
  "kortex_driver/DeleteWifiConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteWifiConfiguration-response)))
  "Returns string type for a service object of type 'DeleteWifiConfiguration-response"
  "kortex_driver/DeleteWifiConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteWifiConfiguration-response>)))
  "Returns md5sum for a message object of type '<DeleteWifiConfiguration-response>"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteWifiConfiguration-response)))
  "Returns md5sum for a message object of type 'DeleteWifiConfiguration-response"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteWifiConfiguration-response>)))
  "Returns full string definition for message of type '<DeleteWifiConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteWifiConfiguration-response)))
  "Returns full string definition for message of type 'DeleteWifiConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteWifiConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteWifiConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteWifiConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteWifiConfiguration)))
  'DeleteWifiConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteWifiConfiguration)))
  'DeleteWifiConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteWifiConfiguration)))
  "Returns string type for a service object of type '<DeleteWifiConfiguration>"
  "kortex_driver/DeleteWifiConfiguration")