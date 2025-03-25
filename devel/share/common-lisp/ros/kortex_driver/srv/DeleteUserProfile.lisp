; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteUserProfile-request.msg.html

(cl:defclass <DeleteUserProfile-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle)))
)

(cl:defclass DeleteUserProfile-request (<DeleteUserProfile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteUserProfile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteUserProfile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteUserProfile-request> is deprecated: use kortex_driver-srv:DeleteUserProfile-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteUserProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteUserProfile-request>) ostream)
  "Serializes a message object of type '<DeleteUserProfile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteUserProfile-request>) istream)
  "Deserializes a message object of type '<DeleteUserProfile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteUserProfile-request>)))
  "Returns string type for a service object of type '<DeleteUserProfile-request>"
  "kortex_driver/DeleteUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteUserProfile-request)))
  "Returns string type for a service object of type 'DeleteUserProfile-request"
  "kortex_driver/DeleteUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteUserProfile-request>)))
  "Returns md5sum for a message object of type '<DeleteUserProfile-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteUserProfile-request)))
  "Returns md5sum for a message object of type 'DeleteUserProfile-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteUserProfile-request>)))
  "Returns full string definition for message of type '<DeleteUserProfile-request>"
  (cl:format cl:nil "UserProfileHandle input~%~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteUserProfile-request)))
  "Returns full string definition for message of type 'DeleteUserProfile-request"
  (cl:format cl:nil "UserProfileHandle input~%~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteUserProfile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteUserProfile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteUserProfile-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteUserProfile-response.msg.html

(cl:defclass <DeleteUserProfile-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteUserProfile-response (<DeleteUserProfile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteUserProfile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteUserProfile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteUserProfile-response> is deprecated: use kortex_driver-srv:DeleteUserProfile-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteUserProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteUserProfile-response>) ostream)
  "Serializes a message object of type '<DeleteUserProfile-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteUserProfile-response>) istream)
  "Deserializes a message object of type '<DeleteUserProfile-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteUserProfile-response>)))
  "Returns string type for a service object of type '<DeleteUserProfile-response>"
  "kortex_driver/DeleteUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteUserProfile-response)))
  "Returns string type for a service object of type 'DeleteUserProfile-response"
  "kortex_driver/DeleteUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteUserProfile-response>)))
  "Returns md5sum for a message object of type '<DeleteUserProfile-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteUserProfile-response)))
  "Returns md5sum for a message object of type 'DeleteUserProfile-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteUserProfile-response>)))
  "Returns full string definition for message of type '<DeleteUserProfile-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteUserProfile-response)))
  "Returns full string definition for message of type 'DeleteUserProfile-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteUserProfile-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteUserProfile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteUserProfile-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteUserProfile)))
  'DeleteUserProfile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteUserProfile)))
  'DeleteUserProfile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteUserProfile)))
  "Returns string type for a service object of type '<DeleteUserProfile>"
  "kortex_driver/DeleteUserProfile")