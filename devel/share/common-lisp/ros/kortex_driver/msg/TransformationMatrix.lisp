; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TransformationMatrix.msg.html

(cl:defclass <TransformationMatrix> (roslisp-msg-protocol:ros-message)
  ((r0
    :reader r0
    :initarg :r0
    :type kortex_driver-msg:TransformationRow
    :initform (cl:make-instance 'kortex_driver-msg:TransformationRow))
   (r1
    :reader r1
    :initarg :r1
    :type kortex_driver-msg:TransformationRow
    :initform (cl:make-instance 'kortex_driver-msg:TransformationRow))
   (r2
    :reader r2
    :initarg :r2
    :type kortex_driver-msg:TransformationRow
    :initform (cl:make-instance 'kortex_driver-msg:TransformationRow))
   (r3
    :reader r3
    :initarg :r3
    :type kortex_driver-msg:TransformationRow
    :initform (cl:make-instance 'kortex_driver-msg:TransformationRow)))
)

(cl:defclass TransformationMatrix (<TransformationMatrix>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformationMatrix>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformationMatrix)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TransformationMatrix> is deprecated: use kortex_driver-msg:TransformationMatrix instead.")))

(cl:ensure-generic-function 'r0-val :lambda-list '(m))
(cl:defmethod r0-val ((m <TransformationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:r0-val is deprecated.  Use kortex_driver-msg:r0 instead.")
  (r0 m))

(cl:ensure-generic-function 'r1-val :lambda-list '(m))
(cl:defmethod r1-val ((m <TransformationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:r1-val is deprecated.  Use kortex_driver-msg:r1 instead.")
  (r1 m))

(cl:ensure-generic-function 'r2-val :lambda-list '(m))
(cl:defmethod r2-val ((m <TransformationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:r2-val is deprecated.  Use kortex_driver-msg:r2 instead.")
  (r2 m))

(cl:ensure-generic-function 'r3-val :lambda-list '(m))
(cl:defmethod r3-val ((m <TransformationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:r3-val is deprecated.  Use kortex_driver-msg:r3 instead.")
  (r3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformationMatrix>) ostream)
  "Serializes a message object of type '<TransformationMatrix>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r3) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformationMatrix>) istream)
  "Deserializes a message object of type '<TransformationMatrix>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r3) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformationMatrix>)))
  "Returns string type for a message object of type '<TransformationMatrix>"
  "kortex_driver/TransformationMatrix")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformationMatrix)))
  "Returns string type for a message object of type 'TransformationMatrix"
  "kortex_driver/TransformationMatrix")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformationMatrix>)))
  "Returns md5sum for a message object of type '<TransformationMatrix>"
  "01b579348ba31feab01058d980795bad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformationMatrix)))
  "Returns md5sum for a message object of type 'TransformationMatrix"
  "01b579348ba31feab01058d980795bad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformationMatrix>)))
  "Returns full string definition for message of type '<TransformationMatrix>"
  (cl:format cl:nil "~%TransformationRow r0~%TransformationRow r1~%TransformationRow r2~%TransformationRow r3~%================================================================================~%MSG: kortex_driver/TransformationRow~%~%float32 c0~%float32 c1~%float32 c2~%float32 c3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformationMatrix)))
  "Returns full string definition for message of type 'TransformationMatrix"
  (cl:format cl:nil "~%TransformationRow r0~%TransformationRow r1~%TransformationRow r2~%TransformationRow r3~%================================================================================~%MSG: kortex_driver/TransformationRow~%~%float32 c0~%float32 c1~%float32 c2~%float32 c3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformationMatrix>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformationMatrix>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformationMatrix
    (cl:cons ':r0 (r0 msg))
    (cl:cons ':r1 (r1 msg))
    (cl:cons ':r2 (r2 msg))
    (cl:cons ':r3 (r3 msg))
))
