% Model & Cables Configuration
function PYCASPR_init(model_name,cables_name)
global model_config modelObj
model_config = ModelConfig(model_name);
modelObj = model_config.getModel(cables_name);
end