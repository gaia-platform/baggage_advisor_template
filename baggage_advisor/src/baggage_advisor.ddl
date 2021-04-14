create table if not exists terminal (
      id string
);

create table if not exists cart_area (
      id string,
      last_cart_id string,
      current_cart_id string,
      current_baggage_id string,
      cart_area__terminal references terminal
);

create table if not exists camera (
      id string,
      function_code string,
      camera_data_code string,
      camera__cart_area references cart_area
);

create table if not exists manifest (
      id string,
      state string,
      desired_qty int32,
      actual_qty int32
);

create table if not exists cart (
      id string,
      shipment references manifest,
      cart__cart_area references cart_area
);

create table if not exists manifest_baggage (
      id string,
      baggage_id string,
      desired_qty int32,
      actual_qty int32,
      manifest_baggage__manifest references manifest
);

create table if not exists baggage (
      id string,
      baggage__manifest references manifest
);
