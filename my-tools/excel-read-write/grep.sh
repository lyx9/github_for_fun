
prediction_keywords="Total Processing Time:"
planning_keywords="task_manager:total"
canbus_keywords="driving_mode"
prediction_xlog="prediction.log.1-1.1"
planning_xlog="planning.log.1-1.1"
canbus_xlog="canbus_proxy.log.1-1.1"

mkdir simple
touch simple/${prediction_xlog}
ls |grep prediction|xargs -i grep "${prediction_keywords}" {} >>simple/${prediction_xlog}

touch simple/${planning_xlog}
ls |grep planning|xargs -i grep  "${planning_keywords}" {} >>simple/${planning_xlog}

#touch simple/${canbus_xlog}
#ls |grep planning|xargs -i grep  ${canbus_keywords} {} >>simple/${canbus_xlog}

#python pnc-time-parse.py -a $1

