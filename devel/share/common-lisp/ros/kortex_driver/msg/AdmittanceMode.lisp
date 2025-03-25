; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude AdmittanceMode.msg.html

(cl:defclass <AdmittanceMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AdmittanceMode (<AdmittanceMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AdmittanceMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AdmittanceMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<AdmittanceMode> is deprecated: use kortex_driver-msg:AdmittanceMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AdmittanceMode>)))
    "Constants for message type '<AdmittanceMode>"
  '((:UNSPECIFIED_ADMITTANCE_MODE . 0)
    (:CARTESIAN . 1)
    (:JOINT . 2)
    (:NULL_SPACE . 3)
    (:DISABLED . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AdmittanceMode)))
    "Constants for message type 'AdmittanceMode"
  '((:UNSPECIFIED_ADMITTANCE_MODE . 0)
    (:CARTESIAN . 1)
    (:JOINT . 2)
    (:NULL_SPACE . 3)
    (:DISABLED . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AdmittanceMode>) ostream)
  "Serializes a message object of type '<AdmittanceMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AdmittanceMode>) istream)
  "Deserializes a message object of type '<AdmittanceMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AdmittanceMode>)))
  "Returns string type for a message object of type '<AdmittanceMode>"
  "kortex_driver/AdmittanceMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AdmittanceMode)))
  "Returns string type for a message object of type 'AdmittanceMode"
  "kortex_driver/AdmittanceMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AdmittanceMode>)))
  "Returns md5sum for a message object of type '<AdmittanceMode>"
  "1a52f2d7eb92e509117483a6b9c79ce1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AdmittanceMode)))
  "Returns md5sum for a message object of type 'AdmittanceMode"
  "1a52f2d7eb92e509117483a6b9c79ce1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AdmittanceMode>)))
  "Returns full string definition for message of type '<AdmittanceMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ADMITTANCE_MODE = 0~%~%uint32 CARTESIAN = 1~%~%uint32 JOINT = 2~%~%uint32 NULL_SPACE = 3~%~%uint32 DISABLED = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AdmittanceMode)))
  "Returns full string definition for message of type 'AdmittanceMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ADMITTANCE_MODE = 0~%~%uint32 CARTESIAN = 1~%~%uint32 JOINT = 2~%~%uint32 NULL_SPACE = 3~%~%uint32 DISABLED = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AdmittanceMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AdmittanceMode>))
  "Converts a ROS message object to a list"
  (cl:list 'AdmittanceMode
))
