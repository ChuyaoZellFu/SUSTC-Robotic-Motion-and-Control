; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetBluetoothEnableState-request.msg.html

(cl:defclass <SetBluetoothEnableState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:BluetoothEnableState
    :initform (cl:make-instance 'kortex_driver-msg:BluetoothEnableState)))
)

(cl:defclass SetBluetoothEnableState-request (<SetBluetoothEnableState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBluetoothEnableState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBluetoothEnableState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetBluetoothEnableState-request> is deprecated: use kortex_driver-srv:SetBluetoothEnableState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetBluetoothEnableState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBluetoothEnableState-request>) ostream)
  "Serializes a message object of type '<SetBluetoothEnableState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBluetoothEnableState-request>) istream)
  "Deserializes a message object of type '<SetBluetoothEnableState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBluetoothEnableState-request>)))
  "Returns string type for a service object of type '<SetBluetoothEnableState-request>"
  "kortex_driver/SetBluetoothEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBluetoothEnableState-request)))
  "Returns string type for a service object of type 'SetBluetoothEnableState-request"
  "kortex_driver/SetBluetoothEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBluetoothEnableState-request>)))
  "Returns md5sum for a message object of type '<SetBluetoothEnableState-request>"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBluetoothEnableState-request)))
  "Returns md5sum for a message object of type 'SetBluetoothEnableState-request"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBluetoothEnableState-request>)))
  "Returns full string definition for message of type '<SetBluetoothEnableState-request>"
  (cl:format cl:nil "BluetoothEnableState input~%~%================================================================================~%MSG: kortex_driver/BluetoothEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBluetoothEnableState-request)))
  "Returns full string definition for message of type 'SetBluetoothEnableState-request"
  (cl:format cl:nil "BluetoothEnableState input~%~%================================================================================~%MSG: kortex_driver/BluetoothEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBluetoothEnableState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBluetoothEnableState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBluetoothEnableState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetBluetoothEnableState-response.msg.html

(cl:defclass <SetBluetoothEnableState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetBluetoothEnableState-response (<SetBluetoothEnableState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBluetoothEnableState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBluetoothEnableState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetBluetoothEnableState-response> is deprecated: use kortex_driver-srv:SetBluetoothEnableState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetBluetoothEnableState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBluetoothEnableState-response>) ostream)
  "Serializes a message object of type '<SetBluetoothEnableState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBluetoothEnableState-response>) istream)
  "Deserializes a message object of type '<SetBluetoothEnableState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBluetoothEnableState-response>)))
  "Returns string type for a service object of type '<SetBluetoothEnableState-response>"
  "kortex_driver/SetBluetoothEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBluetoothEnableState-response)))
  "Returns string type for a service object of type 'SetBluetoothEnableState-response"
  "kortex_driver/SetBluetoothEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBluetoothEnableState-response>)))
  "Returns md5sum for a message object of type '<SetBluetoothEnableState-response>"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBluetoothEnableState-response)))
  "Returns md5sum for a message object of type 'SetBluetoothEnableState-response"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBluetoothEnableState-response>)))
  "Returns full string definition for message of type '<SetBluetoothEnableState-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBluetoothEnableState-response)))
  "Returns full string definition for message of type 'SetBluetoothEnableState-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBluetoothEnableState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBluetoothEnableState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBluetoothEnableState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBluetoothEnableState)))
  'SetBluetoothEnableState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBluetoothEnableState)))
  'SetBluetoothEnableState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBluetoothEnableState)))
  "Returns string type for a service object of type '<SetBluetoothEnableState>"
  "kortex_driver/SetBluetoothEnableState")