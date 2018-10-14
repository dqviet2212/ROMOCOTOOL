#include <iostream>
#include "OCP.h"
#include "NLP.h"
#include "IPOPT.h"
#include "linkedFunction.h"

int main() {
    /**********************/
    OCP ocp;
    ocp.ocpMethod1();
    ocp.ocpMethod2();
    ocp.ocpMethod3();
    /**********************/
    NLP nlp;
    nlp.nlpMethod1();
    nlp.nlpMethod2();
    nlp.nlpMethod3();
    /**********************/
    IPOPT ipopt;
    ipopt.ipoptMethod1();
    ipopt.ipoptMethod2();
    ipopt.ipoptMethod3();
    /**********************/
    linkedFunction();
    return 0;
}
