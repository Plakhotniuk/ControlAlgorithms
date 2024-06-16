//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_BASECONTROLLER_HPP
#define CONTROLALGORITHMS_BASECONTROLLER_HPP

namespace Controllers{

    class BaseController{
    public:
        virtual void computeControl();
        virtual ~BaseController() {};
    };
}


#endif //CONTROLALGORITHMS_BASECONTROLLER_HPP
