classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
    
    methods(Access = public)
        
        function this = GPSMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate(); %Q1c: x_k estimate
            this.errorZ = x(1:2) - this.z; %Q1c:z=x-GPSmeansurement
        end
        
        function linearizeOplus(this)
            %Q1c:compute Jabocian
            this.J{1} = ...
                [1 0 0;
                0 1 0];
        end
    end
end