; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VisionConfig_RotationMatrix.msg.html

(cl:defclass <VisionConfig_RotationMatrix> (roslisp-msg-protocol:ros-message)
  ((row1
    :reader row1
    :initarg :row1
    :type kortex_driver-msg:VisionConfig_RotationMatrixRow
    :initform (cl:make-instance 'kortex_driver-msg:VisionConfig_RotationMatrixRow))
   (row2
    :reader row2
    :initarg :row2
    :type kortex_driver-msg:VisionConfig_RotationMatrixRow
    :initform (cl:make-instance 'kortex_driver-msg:VisionConfig_RotationMatrixRow))
   (row3
    :reader row3
    :initarg :row3
    :type kortex_driver-msg:VisionConfig_RotationMatrixRow
    :initform (cl:make-instance 'kortex_driver-msg:VisionConfig_RotationMatrixRow)))
)

(cl:defclass VisionConfig_RotationMatrix (<VisionConfig_RotationMatrix>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionConfig_RotationMatrix>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionConfig_RotationMatrix)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VisionConfig_RotationMatrix> is deprecated: use kortex_driver-msg:VisionConfig_RotationMatrix instead.")))

(cl:ensure-generic-function 'row1-val :lambda-list '(m))
(cl:defmethod row1-val ((m <VisionConfig_RotationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:row1-val is deprecated.  Use kortex_driver-msg:row1 instead.")
  (row1 m))

(cl:ensure-generic-function 'row2-val :lambda-list '(m))
(cl:defmethod row2-val ((m <VisionConfig_RotationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:row2-val is deprecated.  Use kortex_driver-msg:row2 instead.")
  (row2 m))

(cl:ensure-generic-function 'row3-val :lambda-list '(m))
(cl:defmethod row3-val ((m <VisionConfig_RotationMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:row3-val is deprecated.  Use kortex_driver-msg:row3 instead.")
  (row3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionConfig_RotationMatrix>) ostream)
  "Serializes a message object of type '<VisionConfig_RotationMatrix>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'row1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'row2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'row3) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionConfig_RotationMatrix>) istream)
  "Deserializes a message object of type '<VisionConfig_RotationMatrix>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'row1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'row2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'row3) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionConfig_RotationMatrix>)))
  "Returns string type for a message object of type '<VisionConfig_RotationMatrix>"
  "kortex_driver/VisionConfig_RotationMatrix")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionConfig_RotationMatrix)))
  "Returns string type for a message object of type 'VisionConfig_RotationMatrix"
  "kortex_driver/VisionConfig_RotationMatrix")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionConfig_RotationMatrix>)))
  "Returns md5sum for a message object of type '<VisionConfig_RotationMatrix>"
  "37bab3875ca8eda43ce39de02fa3b72b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionConfig_RotationMatrix)))
  "Returns md5sum for a message object of type 'VisionConfig_RotationMatrix"
  "37bab3875ca8eda43ce39de02fa3b72b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionConfig_RotationMatrix>)))
  "Returns full string definition for message of type '<VisionConfig_RotationMatrix>"
  (cl:format cl:nil "~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionConfig_RotationMatrix)))
  "Returns full string definition for message of type 'VisionConfig_RotationMatrix"
  (cl:format cl:nil "~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionConfig_RotationMatrix>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'row1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'row2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'row3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionConfig_RotationMatrix>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionConfig_RotationMatrix
    (cl:cons ':row1 (row1 msg))
    (cl:cons ':row2 (row2 msg))
    (cl:cons ':row3 (row3 msg))
))
